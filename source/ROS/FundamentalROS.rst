Fundamental ROS Entities
------------------------

Most of the concepts in ROS are based around a set of fundamental
entities, which will be discussed in this section. Understanding ROS,
the challenges which ROS attempts to overcome and the challenges of
using ROS are not possible without a firm understanding of these
materials.

Package
~~~~~~~

Pieces of software are, as the name suggests, known as packages in ROS.
Each package carries out a single or group of tightly related functions.
Packages need not include code at all; some packages are simply
metapackages for the purpose of ensuring that other packages are present
on the system, while other packages include startup routines for robots
or 3-D physical definitions which are used to render robots in
simulation.

Packages may be placed in different ROS environments, and a number of
these environments may be exposed simultaneously, allowing developers to
switch between different groups of packages with ease.

Node
~~~~

The node is quite possibly the single most important concept to
understand when discussing the use of ROS. Nodes are essentially
vertices in the computation graph that is implemented in ROS, and all of
the computation in ROS occurs in a node.

It is considered to be the best form in ROS for a single node to carry
out a single task. This helps to promote code reuse, as the node could
then be used to perform the same task as part of a completely different
system, ideally without any modification of the code.

It is quite common to see hundreds of nodes as part of a single ROS
environment, and it is also common of many of them to be active
simultaneously. For instance:

Master
~~~~~~

The ROS master provides a registration system for the nodes on a ROS
system, among other services. Think of it as the operator of a phone
network. When a node requests information, it asks the ROS master to
connect it to someone who can provide that information. The ROS master
doesn’t actually give the information to the node, it simply tells it
where it may be found. This communication happens over an XMLRPC
protocol.

A node does not typically communicate with the master once it has
finished initializing and is sending and receiving data. It does,
however, talk to the master whenever it needs a new data stream or
parameter information.

Worth noting is that while communication between the nodes and the
master is sparse, loss of communication with the master can be
devastating to a ROS system. If the master were to crash or become
otherwise unavailable, the entire ROS system would likely fail if any
master communication were to be attempted. The node which tried to
communicate with the master would fail, likely causing a domino effect
in nodes trying to request data streams that are sequentially becoming
unavailable.

In general, every ROS system must have exactly one master. There exist
methods of inter-master communication, but there is no built-in
methodology for this.

Message
~~~~~~~

Any data or information that is exchanged between nodes is known as a
message, which is defined as a combination of primitive data types or
other messages. Some messages include a common header, which includes a
sequence number, time stamp and a physical origin known as a frame ID.
For example, a “Twist” message contains 6 Float64 values; a 3-D vector
of linear velocities as well as a 3-D vector of angular velocities. This
message is widely used to describe the velocity of a body in ROS.

Any message defined in ROS is available in any of the supported language
in ROS. Once a node sends a message over ROS, the message can be
interpreted by another node even if the nodes are not written in the
same language or are running on the same operating system. On that note,
messages could be considered to be “data contracts” among nodes.

Topic
~~~~~

While a node may request a certain type of data from the master, it is
possible that multiple nodes could provide data of that type. The use of
“topics” is necessary to uniquely identify a data source to other nodes.
Therefore, when a node notifies the master of available data, it must
provide a topic name for that data. A connection between nodes is only
ever established if the nodes agree on a data type and a topic.

Topics can be thought of as being similar to a telephone number. When a
node registers its “number” with the master (a process known as
advertising), the master notes the message type that the “number”
corresponds to as well as the network address of the node that is
providing it. When another node “calls” that number (a process known as
subscribing), the master looks to see if there is a registered node
providing the requested message type, and tells the node what the
address of the other node is. The direct connection between the nodes is
then established and the data transfer begins.

It should be noted that while this seems to indicate that a topic
corresponds to a single server-client relationship, the topic system
allows for multiple subscribers as well as multiple publishers.
Therefore the relationship is generally referred to as
publisher-subscriber, or “pub-sub.”

Service
~~~~~~~

The publisher-subscriber (pub-sub) model is not always appropriate for
all types of data, and the service system exists in ROS to fill in the
gap. Services are, like pub-sub messages, exposed in ROS over topics.
The group of topic names is separate from the pub-sub topics, but the
structure remains.

Services are unique in that they are based on a call-and-response model
instead of pub-sub. A node can not only request information from another
node, but it can include a message to the other node containing
information about the request. The remote node then responds with a
single message back to the node that initiated the service call.

Services are useful in many ways, but should not be over-used. Each time
a service call is made, the node must request the address of the service
provider from the master. If service calls are made frequently, a
bottleneck could form in the computation graph at the master.
