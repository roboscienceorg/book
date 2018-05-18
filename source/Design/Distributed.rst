Distributed Computation and Communications
------------------------------------------

Sockets provide a bidirectional channel between two processes. Although
one side was setup like a server and one side like a client, this was
basically a point to point type of communication. With only two
processes one could call this peer to peer or client server as well,
however, in this case it is strictly one process to one process. The
socket mechanism underneath is used to implement client server
architectures. This allows many processes to connect into a single
process. So a client server architecture is immediately available.

Robotics is evolving from having completely integrated monolithic
control systems to modular distributed architectures. As the hardware
becomes more powerful and the goals more sophisticated, the complexity
of the control system increases. It is increasing in a superlinear
manner. We may view the workings of robotics software as a collection of
interconnected computations and view the collection in a graph. Nodes
would represent computational blocks, specifically processes.
Interprocess communication is represented by the edges connecting the
nodes in the graph. The connection between two nodes is the point to
point communication we discussed above. A single robot could have many
nodes. Some that control low level aspects like drive motors or wheel
encoder data. Others higher level like processing data for computer
vision algorithms, estimating position or routing the robot over the
landscape.

Many of the nodes will be producing data for other nodes. Some nodes are
producers, some consumers and some are both. The underlying client
server architecture appears to be required. For a particular node that
produces data for several other nodes, it needs to be a server to those
client nodes. With multiple servers running and each delivering a
different service, how should we manage this? The IP system connects to
a host:port combination. So, one would need to know the host:port pair
apriori. The host name might be known, but what about ports? A system
could have external software that uses any particular port range. Having
the vast collection of sensors and user contributed computational nodes
means that a port numbering and classification system needs to be
devised. Unix systems used to have remote procedures bound to port
numbers. This works when there is a limited list. When the list gets
long we need something like the Dewey Decimal system in the library. Of
course we know that as the scale of node types increases, the
predetermined mapping will eventually break.

One might think to run everything through a central server. Of course
this produces a significant bottleneck and will not scale at all. A
centralized system will not work. The system needs to be dynamic and
configurable. However, we need a way to allow the data producer to
connect with the data consumer. A peer to peer connection is desired to
avoid bottlenecks and other network issues related to a single central
server. We also need a way to dynamically map hosts and ports as the
system needs. This means that a database is required. The information
can be centralized or distributed. If scale allows, a centralized system
will have better response bounds since we know exactly how long it will
take to find the required data. A distributed database may require
several requests to get the information.

When a service starts up, it should register itself in a publicly
available database. It would register with a central server and record
that a particular service may be found at host:port pair. When the
client is ready, it can query the central repository, request the
service location and then connect to the correct server. This main
server or master is a nameserver. Having only the job of handing out
names at the start of the service, it does not affect the communications
later on. We will say a particular node with data ( the service) will
publish this data. This means that it registers with the name server and
accepts connections. A client requiring the data will subscript to the
data by requested the publishing node from the nameserver and then
requesting a connection to that node.

Having a specific service with multiple clients can complicate matters.
The point of the service is to produce something, not worry about
communications. So, to address this, a publish-subscribe mechanism can
be built that treats the data as a topic. That topic is available on a
type of message bus. The publisher and subscriber should be separated
and not know about each other. This way one can deal with issues of
scale, broken connections, reconnections and other real world issues
without disturbing either the publisher or subscriber. Of course this
will eliminate request-reply types of communication which should be
addressed using a direct point to point type of channel. These ideas
will be fully developed when we cover the Robot Operating System.

Before we proceed with building robots, we need to discuss safety, human
interaction and human environments. Robots can be very helpful, capable
even lifesaving devices. However they can pose serious risks which need
to be recognized and addressed. In addition, there are complexities in
working with humans and in human environments which need to be addressed
as well. In this short chapter we examine some of the issues.
