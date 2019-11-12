from os import listdir
from os.path import isfile, join
from tensorflow.core.framework import graph_pb2

import argparse
import re
import tensorflow as tf

# Name of the iterator node which passes input to the first convolution layer
# While freezing, we go through the graph and replace the iterator nodes with the input node
kIteratorName = "IteratorGetNext"
kInputName = "input"

def main(args):
    """
    Checkpoints are stored as a combination of index, meta and data files
    Freeze the most recent checkpoint into a protobuf file

    Arguments:
    args: the parsed command line arguments
    """
    # Get the file path for the most recent meta
    meta_filename = get_latest_meta(args.checkpoint_dir)
    meta_path = join(args.checkpoint_dir, meta_filename)
    # Get output node name and full path for the output file of the frozen graph
    output_node_names = [args.output_nodename]
    output_file_path = join(args.checkpoint_dir, args.output_filename)
    with tf.Session(config=tf.ConfigProto(allow_soft_placement=True)) as sess:
        # Restore the graph
        saver = tf.train.import_meta_graph(meta_path)
        # Load weights
        saver.restore(sess, tf.train.latest_checkpoint(args.checkpoint_dir))
        # Freeze the graph
        frozen_graph_def = tf.graph_util.convert_variables_to_constants(
            sess, sess.graph_def, output_node_names)
        # Create a graph with an input placeholder,
        with tf.Graph().as_default() as placeholder:
            input = tf.placeholder(tf.float32, name=kInputName, shape=[1, 256, 512, 3])
        # Create a graph definition and add the input placeholder to it
        output_graph_def = graph_pb2.GraphDef()
        output_graph_def.node.extend(placeholder.as_graph_def().node[:1])
        # Extend the new graph definition with all the nodes from the old one except the iterators
        output_graph_def.node.extend(frozen_graph_def.node[1:])
        # Make sure none of the remaining nodes accept input from an iterator node
        for node in output_graph_def.node:
            if kIteratorName in node.input:
                for i in range(len(node.input)):
                    if node.input[i] == kIteratorName:
                        node.input[i] = kInputName
        # Save the frozen graph
        with open(output_file_path, 'wb') as f:
            f.write(output_graph_def.SerializeToString())


def get_latest_meta(checkpoint_dir):
    """
    Find the most recent meta file in the checkpoint directory
    Checkpoints are stored in the form of three files with extensions as follows:
    * .meta: Stores the graph structure (eg.: model.ckpt-100.meta)
    * .data: Stores the values of the variables saved (eg.: model.ckpt-100.data-00000-of-00001)
    * .index: Stores the list of variable names and shapes (eg.: model.ckpt-100.index)
    The meta file is neccessary to build the model graph before loading the saved weights

    Arguments:
    checkpoint_dir: the path to the checkpoint directory

    Returns:
    Filename of the most recently checkpointed meta file
    """
    # List all meta files in checkpoint directory
    meta_files = [
        f for f in listdir(checkpoint_dir) if isfile(join(checkpoint_dir, f)) and "meta" in f
    ]
    # List corresponding step numbers in their respective filenames
    meta_indices = list(map(lambda x: int(re.findall('\d+', x)[0]), meta_files))
    # Find the one with the largest index
    meta_filename = meta_files[meta_indices.index(max(meta_indices))]
    return meta_filename


def parse_arguments():
    """
    Set required input arguments and add functionality to parse them

    Returns:
    The arugments parsed from the command line
    """
    parser = argparse.ArgumentParser()
    parser.add_argument("--checkpoint_dir", help="Path to the output pb directory")
    parser.add_argument("--output_filename", help="Output file name")
    parser.add_argument("--output_nodename", help="Output node name")
    args = parser.parse_args()
    return args


if __name__ == '__main__':
    args = parse_arguments()
    main(args)
