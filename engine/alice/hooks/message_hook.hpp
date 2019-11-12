/*
Copyright (c) 2018, NVIDIA CORPORATION. All rights reserved.

NVIDIA CORPORATION and its licensors retain all intellectual property
and proprietary rights in and to this software, related documentation
and any modifications thereto. Any use, reproduction, disclosure or
distribution of this software and related documentation without an express
license agreement from NVIDIA CORPORATION is strictly prohibited.
*/
#pragma once

#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "engine/alice/components/MessageLedger.hpp"
#include "engine/alice/hooks/hook.hpp"
#include "engine/alice/message.hpp"
#include "engine/core/assert.hpp"
#include "engine/core/optional.hpp"

namespace isaac {
namespace alice {

// Base class for a RX or TX hook which connects to a MessageLedger for message passing
class MessageHook : public Hook {
 public:
  virtual ~MessageHook() = default;
  MessageHook(Component* component, const std::string& tag);
  // The tag given to a message received or transmitted
  const std::string& tag() const { return tag_; }
  // The ledger to which this hook is connected
  MessageLedger* ledger() const { return ledger_; }
  // A unique channel ID constructed from the component UUID and the tag
  std::string channel_id() const;
  // Returns true if this hook is for a receiving message endpoint
  virtual bool isReceiving() const = 0;

 private:
  friend class Component;

  void connect() override;

  std::string tag_;
  MessageLedger* ledger_;
};

// Base class for RX hooks to receive messages
class RxMessageHook : public MessageHook {
 public:
  virtual ~RxMessageHook() = default;
  RxMessageHook(Component* component, const std::string& tag) : MessageHook(component, tag) {}

  bool isReceiving() const override { return true; }

  // Checks if there is currently a message available
  bool available() const { return message_.get() != nullptr; }
  // Gets the uuid of the current message
  const Uuid& message_uuid() const { return base_message()->uuid; }
  // The acquisition time of the current message
  int64_t acqtime() const { return base_message()->acqtime; }
  // The publish time of the current message
  int64_t pubtime() const { return base_message()->pubtime; }

  // For internal use only. TODO: protect this better
  void setMessage(ConstMessageBasePtr message) { message_ = message; }
  // Expose base_message() to Codelet component. For internal use only. TODO: protect this better
  const MessageBase* getMessage() { return base_message(); }

  // Gets buffers associated with this message
  const std::vector<SharedAlignedBuffer>& buffers() const { return message_->buffers; }

 protected:
  const MessageBase* base_message() const {
    ASSERT(available(), "No message available. Use `available` to check before accessing.");
    return message_.get();
  }

  // Executes the given callbacke for all new messages on this channel.
  void processAllNewMessagesImpl(std::function<void(const ConstMessageBasePtr&)> callback) const;

 private:
  ConstMessageBasePtr message_;
};

// Base class for TX hooks to transmit messages
class TxMessageHook : public MessageHook {
  friend class PyCodelet;  // PyCodelet needs to access the publishImpl to publish without templated
                           // TxMessageHook's
 public:
  virtual ~TxMessageHook() = default;
  TxMessageHook(Component* component, const std::string& tag) : MessageHook(component, tag) {}

  bool isReceiving() const override { return false; }

  // Gets reference to buffers associated with the message which will be created
  std::vector<SharedAlignedBuffer>& buffers() { return buffers_; }

 protected:
  // Publishes a message to the message ledger with either given or current acquisition time
  void publishImpl(MessageBasePtr message, std::optional<int64_t> acqtime = std::nullopt);

 private:
  std::vector<SharedAlignedBuffer> buffers_;
};

// An RX message receiver for raw messages
template <typename T>
class RawRx : public RxMessageHook {
 public:
  // Disallow copy
  RawRx(const RawRx&) = delete;
  RawRx& operator=(const RawRx&) = delete;

  RawRx(Component* component, const std::string& tag) : RxMessageHook(component, tag) {}

  // Gets the data block of the current message
  const T& get() const {
    ASSERT(available(), "No message available");
    auto* message = dynamic_cast<const RawMessage<T>*>(base_message());
    ASSERT(message != nullptr, "Message is not a raw message");
    return message->data;
  }

  // A type for callbacks to be used when processing new messages. The callback takes three
  // parameters:
  //   (1) The raw message object of the desired type
  //   (2) The publish time (pubtime) of the message
  //   (3) The acquisition time (acqtime) of the message
  using NewMessageCallback = std::function<void(T, int64_t, int64_t)>;

  // Calls the given callback for all new messages on this channel. Be careful when using this
  // function as it can lead to data congestion.
  void processAllNewMessages(NewMessageCallback callback) const {
    processAllNewMessagesImpl([=](const ConstMessageBasePtr& message_ptr) {
      auto* raw_message = dynamic_cast<const RawMessage<T>*>(message_ptr.get());
      ASSERT(raw_message != nullptr, "Message is not a raw message");
      callback(raw_message->data, message_ptr->pubtime, message_ptr->acqtime);
    });
  }
};

// An TX message transmitter for raw messages
template <typename T>
class RawTx : public TxMessageHook {
 public:
  // Disallow copy
  RawTx(const RawTx&) = delete;
  RawTx& operator=(const RawTx&) = delete;

  RawTx(Component* component, const std::string& tag) : TxMessageHook(component, tag) {}

  // Publish a message with given data and use the current time as acquisition time
  void publish(T data) {
    auto msg = std::make_shared<RawMessage<T>>();
    msg->data = std::move(data);
    publishImpl(msg);
  }
  // Publish a message with given data and acquisition time
  void publish(T data, int64_t acqtime) {
    auto msg = std::make_shared<RawMessage<T>>();
    msg->data = std::move(data);
    publishImpl(msg, acqtime);
  }
};

// An RX message receiver for Cap'n'proto messages
template <typename Proto>
class ProtoRx : public RxMessageHook {
 public:
  // Disallow copy
  ProtoRx(const ProtoRx&) = delete;
  ProtoRx& operator=(const ProtoRx&) = delete;

  ProtoRx(Component* component, const std::string& tag) : RxMessageHook(component, tag) {}

  // Gets a proto reader based on the current message
  typename Proto::Reader getProto() const {
    ASSERT(available(), "No message available");
    auto* proto_message = dynamic_cast<const ProtoMessageBase*>(base_message());
    ASSERT(proto_message != nullptr, "Message is not a proto message");
    return proto_message->reader().getRoot<Proto>();
  }

  // A type for callbacks to be used when processing new messages. The callback takes three
  // parameters:
  //   (1) The message proto object of the desired type
  //   (2) The publish time (pubtime) of the message
  //   (3) The acquisition time (acqtime) of the message
  using NewMessageCallback = std::function<void(typename Proto::Reader, int64_t, int64_t)>;

  // Calls the given callback for all new messages on this channel. Be careful when using this
  // function as it can lead to data congestion.
  void processAllNewMessages(NewMessageCallback callback) const {
    ledger()->readAllNew({component(), tag()}, [=](const ConstMessageBasePtr& message_ptr) {
      auto* proto_message = dynamic_cast<const ProtoMessageBase*>(message_ptr.get());
      ASSERT(proto_message != nullptr, "Message is not a proto message");
      callback(proto_message->reader().getRoot<Proto>(), message_ptr->pubtime,
               message_ptr->acqtime);
    });
  }

  // Marks all new messages as read but calls the callback only for the latest.
  void processLatestNewMessage(NewMessageCallback callback) const {
    ledger()->readLatestNew({component(), tag()}, [=](const ConstMessageBasePtr& message_ptr) {
      auto* proto_message = dynamic_cast<const ProtoMessageBase*>(message_ptr.get());
      ASSERT(proto_message != nullptr, "Message is not a proto message");
      callback(proto_message->reader().getRoot<Proto>(), message_ptr->pubtime,
               message_ptr->acqtime);
    });
  }
  // A type for callbacks to be used when processing new messages. The callback takes three
  // parameters:
  //   (1) The message proto object of the desired type
  //   (2) The message buffers for larger data types.
  //   (3) The publish time (pubtime) of the message
  //   (4) The acquisition time (acqtime) of the message
  using NewMessageBufferedCallback = std::function<void(
      typename Proto::Reader, const std::vector<SharedAlignedBuffer>& buffers, int64_t, int64_t)>;

  // Calls the given callback for all new messages on this channel. Be careful when using this
  // function as it can lead to data congestion.
  void processAllNewMessagesBuffered(NewMessageBufferedCallback callback) const {
    ledger()->readAllNew({component(), tag()}, [=](const ConstMessageBasePtr& message_ptr) {
      auto* proto_message = dynamic_cast<const ProtoMessageBase*>(message_ptr.get());
      ASSERT(proto_message != nullptr, "Message is not a proto message");
      callback(proto_message->reader().getRoot<Proto>(), message_ptr->buffers, message_ptr->pubtime,
               message_ptr->acqtime);
    });
  }

  // Marks all new messages as read but calls the callback only for the latest.
  void processLatestNewMessageBuffered(NewMessageBufferedCallback callback) const {
    ledger()->readLatestNew({component(), tag()}, [=](const ConstMessageBasePtr& message_ptr) {
      auto* proto_message = dynamic_cast<const ProtoMessageBase*>(message_ptr.get());
      ASSERT(proto_message != nullptr, "Message is not a proto message");
      callback(proto_message->reader().getRoot<Proto>(), message_ptr->buffers, message_ptr->pubtime,
               message_ptr->acqtime);
    });
  }
};

// An TX message transmitter for Cap'n'proto messages
template <typename Proto>
class ProtoTx : public TxMessageHook {
 public:
  // Disallow copy
  ProtoTx(const ProtoTx&) = delete;
  ProtoTx& operator=(const ProtoTx&) = delete;

  ProtoTx(Component* component, const std::string& tag) : TxMessageHook(component, tag) {}

  // Starts a new message and returns a builder for it. You must use initProto before publish.
  typename Proto::Builder initProto() {
    capnp_message_builder_.reset(new ::capnp::MallocMessageBuilder());
    return capnp_message_builder_->initRoot<Proto>();
  }

  // Publishes the message which is being built and use the current time as acquisition time
  void publish() { publishImpl(createMessage()); }
  // Publishes the message which is being built with the given aquisition time
  void publish(int64_t acqtime) { publishImpl(createMessage(), acqtime); }

 private:
  MessageBasePtr createMessage() {
    ASSERT(capnp_message_builder_, "Must create proto using the initProto function");
    auto msg = std::make_shared<MallocProtoMessage>(std::move(capnp_message_builder_));
    capnp_message_builder_ = std::unique_ptr<::capnp::MallocMessageBuilder>();
    return msg;
  }

  std::unique_ptr<::capnp::MallocMessageBuilder> capnp_message_builder_;
};

// Connects a transmitter to a receiver
void Connect(Component* tx, const std::string& tx_tag, Component* rx, const std::string& rx_tag);
// Connects an TX transmitter hook to an RX receiver hook (raw version)
template <typename T>
void Connect(RawTx<T>& tx, const RawRx<T>& rx) {
  Connect(tx.component(), tx.tag(), rx.component(), rx.tag());
}
// Connects a transmitter to a RX hook
template <typename P>
void Connect(Component* tx, const std::string& tx_tag, const RawRx<P>& rx) {
  Connect(tx, tx_tag, rx.component(), rx.tag());
}
// Connects a TX hook to a receiver
template <typename P>
void Connect(RawTx<P>& tx, Component* rx, const std::string& rx_tag) {
  Connect(tx.component(), tx.tag(), rx, rx_tag);
}
// Connects an TX transmitter hook to an RX receiver hook (proto version)
template <typename P>
void Connect(ProtoTx<P>& tx, const ProtoRx<P>& rx) {
  Connect(tx.component(), tx.tag(), rx.component(), rx.tag());
}
// Connects a transmitter to a RX hook
template <typename P>
void Connect(Component* tx, const std::string& tx_tag, const ProtoRx<P>& rx) {
  Connect(tx, tx_tag, rx.component(), rx.tag());
}
// Connects a TX hook to a receiver
template <typename P>
void Connect(ProtoTx<P>& tx, Component* rx, const std::string& rx_tag) {
  Connect(tx.component(), tx.tag(), rx, rx_tag);
}

}  // namespace alice
}  // namespace isaac

#define ISAAC_RAW_RX(TYPE, NAME)                         \
 private:                                                \
  ::isaac::alice::RawRx<TYPE> rx_##NAME##_{this, #NAME}; \
                                                         \
 public:                                                 \
  const ::isaac::alice::RawRx<TYPE>& rx_##NAME() const { return rx_##NAME##_; }
#define ISAAC_RAW_TX(TYPE, NAME)                         \
 private:                                                \
  ::isaac::alice::RawTx<TYPE> tx_##NAME##_{this, #NAME}; \
                                                         \
 public:                                                 \
  ::isaac::alice::RawTx<TYPE>& tx_##NAME() { return tx_##NAME##_; }
// NOLINT

#define ISAAC_PROTO_RX(PROTO, NAME)                         \
 private:                                                   \
  ::isaac::alice::ProtoRx<PROTO> rx_##NAME##_{this, #NAME}; \
                                                            \
 public:                                                    \
  const ::isaac::alice::ProtoRx<PROTO>& rx_##NAME() const { return rx_##NAME##_; }
#define ISAAC_PROTO_TX(PROTO, NAME)                         \
 private:                                                   \
  ::isaac::alice::ProtoTx<PROTO> tx_##NAME##_{this, #NAME}; \
                                                            \
 public:                                                    \
  ::isaac::alice::ProtoTx<PROTO>& tx_##NAME() { return tx_##NAME##_; }
// NOLINT
