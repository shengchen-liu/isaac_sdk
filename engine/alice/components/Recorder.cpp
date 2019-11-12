/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#include "Recorder.hpp"

#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "engine/alice/application.hpp"
#include "engine/alice/backend/backend.hpp"
#include "engine/alice/components/MessageLedger.hpp"
#include "engine/core/assert.hpp"
#include "engine/gems/cask/cask.hpp"
#include "engine/gems/serialization/blob.hpp"
#include "engine/gems/serialization/header.hpp"
#include "messages/alice.capnp.h"
#include "messages/uuid.hpp"

namespace isaac {
namespace alice {

Recorder::Recorder() {}
Recorder::~Recorder() {}

void Recorder::initialize() {
  // Messages received by Recorder will be written to the log
  MessageLedger* ledger = node()->getComponent<MessageLedger>();
  ASSERT(ledger, "Replay requires MessageLedger component");
  ledger->addOnConnectAsRxCallback(
    [ledger, this](const MessageLedger::Endpoint& tx, const MessageLedger::Endpoint& rx) {
      if (rx.component != this) {
        return;
      }
      // notify about replayed messages
      ledger->addOnMessageCallback(rx, tx.component,
          [this, tx, rx](ConstMessageBasePtr message) {
            this->log(tx.component, rx.tag, message);
          });
    });
}

void Recorder::start() {
  // TODO putting this here will not allow us to record data before starting the network
  openCask();
}

void Recorder::stop() {
  // TODO maybe close cask once no recorder needs it anymore
}

void Recorder::deinitialize() {
}

void Recorder::openCask() {
  std::string root = get_base_directory() + "/" + node()->app()->uuid().str();
  const std::string log_tag = get_tag();
  if (!log_tag.empty()) {
    root = root + "/" + log_tag;
  }
  // Close old cask before starting a new one
  if (cask_) {
    ASSERT(cask_->getRoot() != root, "Recorder cannot reuse the same log directory");
    for (auto& it : component_key_to_uuid_) {
      cask_->seriesClose(it.second);
    }
    component_key_to_uuid_.clear();
    cask_->close();
  }
  cask_ = std::make_unique<cask::Cask>(root, cask::Cask::Mode::Write);
}

void Recorder::log(const Component* component, const std::string& key,
                   ConstMessageBasePtr message) {
  if (!get_enabled()) {
    return;
  }
  if (!cask_) {
    LOG_ERROR("Logging before start is currently not supported");
    return;
  }
  const ProtoMessageBase* protomsg = dynamic_cast<const ProtoMessageBase*>(message.get());
  ASSERT(protomsg != nullptr, "Only proto messages are supported for logging");
  std::vector<ByteBufferConstView> segments;
  serialization::CapnpArraysToBlobs(protomsg->segments(), segments);
  // Get segments for message buffers
  std::vector<ByteBufferConstView> buffer_blobs;
  for (const auto& buffer : message->buffers) {
    buffer_blobs.push_back(
        ByteBufferConstView{buffer.host_buffer().begin(), buffer.host_buffer().size()});
  }
  // create message header
  serialization::Header header;
  header.timestamp = message->pubtime;
  header.acqtime = message->acqtime;
  serialization::BlobsToLengths32u(segments, header.segments);
  serialization::BlobsToLengths32u(buffer_blobs, header.buffers);
  // compute header length
  size_t header_length;
  if (!Size(header, true, &header_length, nullptr)) {
    PANIC("could not compute header size");
  }
  // compute total segment length
  const size_t segment_length = serialization::AccumulateLength(segments);
  const size_t buffer_length = serialization::AccumulateLength(buffer_blobs);
  // write data
  cask_->keyValueWrite(message->uuid, header_length + segment_length + buffer_length,
      [&] (uint8_t* begin, uint8_t* end) {
        begin = Serialize(header, begin, end);
        ASSERT(begin, "error serializing header");
        begin = serialization::CopyAll(segments, begin, end);
        begin = serialization::CopyAll(buffer_blobs, begin, end);
      });
  // add to series
  auto compkey = std::pair<Uuid, std::string>{component->uuid(), key};
  auto it = component_key_to_uuid_.find(compkey);
  if (it == component_key_to_uuid_.end()) {
    Uuid uuid = Uuid::Generate();
    it = component_key_to_uuid_.insert({compkey, uuid}).first;
    cask_->seriesOpen(uuid, 24);
    writeChannelIndex();
  }
  serialization::Header uuid_ts;
  uuid_ts.timestamp = message->pubtime;
  uuid_ts.uuid = message->uuid;
  std::array<uint8_t, 24> keybytes;
  const uint32_t flags = serialization::TIP_1_TIMESTAMP | serialization::TIP_2_UUID;
  SerializeWithoutTip(uuid_ts, flags, keybytes.data(), keybytes.data() + keybytes.size());
  cask_->seriesAppend(it->second, ByteBufferConstView{keybytes.data(), keybytes.size()});
}

void Recorder::writeChannelIndex() {
  // the proto
  ::capnp::MallocMessageBuilder header_builder;
  auto index = header_builder.initRoot<MessageChannelIndexProto>();
  auto channels = index.initChannels(component_key_to_uuid_.size());
  size_t counter = 0;
  for (const auto& kvp : component_key_to_uuid_) {
    auto channel = channels[counter++];
    ToProto(kvp.first.first, channel.initComponentUuid());
    channel.setTag(kvp.first.second);
    ToProto(kvp.second, channel.initSeriesUuid());
  }
  std::vector<ByteBufferConstView> blobs;
  serialization::CapnpArraysToBlobs(header_builder.getSegmentsForOutput(), blobs);
  // the header
  serialization::Header header;
  header.segments.reserve(blobs.size());
  for (const auto& blob : blobs) {
    header.segments.push_back(blob.size());
  }
  std::vector<uint8_t> buffer;
  Serialize(header, buffer);
  // write all segments (including the header)
  blobs.insert(blobs.begin(), ByteBufferConstView{buffer.data(), buffer.size()});
  cask_->keyValueWrite(Uuid::FromAsciiString("msg_chnl_idx"), blobs);
}

size_t Recorder::numChannels() {
  MessageLedger* ledger = node()->getComponent<MessageLedger>();
  return ledger ? ledger->numSourceChannels() : 0;
}

}  // namespace alice
}  // namespace isaac
