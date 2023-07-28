ZeroMQ
-----------

ZeroMQ is an open-source universal messaging library. It will provide a clean and portable communication API that sits on top of the basic socket implementation. For us, it means that we can focus our attention on the communications without getting mired down in the details of the way the socket is implemented in the language and operating system. It will help us develop portable code as well as more robust code. ZeroMQ supports the various communication styles we have discussed (and more).

At this point you may ask, “why not use ROS”? ROS indeed will do what we need and is vastly popular in the robotics community. This book is about concepts. With a few exceptions, we are going to develop the programs we need. We do not need the whole ROS ecosystem. We need interprocess communications; we need some type of messaging system. ROS is large. ROS is under active development and can, at times, be challenging to install as well as use. ZeroMQ is much smaller, with bindings for many languages and, for us, is a library available to the Julia interpreter (as well as many other languages).

We are only going to introduce ZeroMQ. To really learn about it, especially for applications outside what we need, please refer to the Guide: http://zguide.zeromq.org/ . The guide is mostly examples in C, but there are some Julia examples which can be used as starting code. In addition there are examples from other language such as Python which address ZeroMQ: https://pyzmq.readthedocs.io/en/latest/ and https://learning-0mq-with-pyzmq.readthedocs.io/en/latest/ .

The easiest to understand is the REP pattern. It is a direct peer to peer client server communication pattern. This is known as REQUEST - REPLY.

.. _`fig:paircomm`:
.. figure:: ToolsFigures/paircomm.*
   :width: 70%
   :align:  center

   Pair Communication using ZMQ.


A basic example is taken from the ZeroMQ guide. Here is a client server example. The Server code:

.. code-block:: julia
   :dedent: 1

    #!/usr/bin/env julia

    #
    # Hello World server in Julia
    # Binds REP socket to tcp://*:5555
    # Expects "Hello" from client, replies "World"
    #

    using ZMQ

    context = Context()
    socket = Socket(context, REP)
    ZMQ.bind(socket, "tcp://*:5555")

    while true
        # Wait for next request from client
        message = String(ZMQ.recv(socket))
        println("Received request: $message")

        # Do some 'work'
        sleep(1)

        # Send reply back to client
        ZMQ.send(socket, "World")
    end

    # classy hit men always clean up when finish the job.
    ZMQ.close(socket)
    ZMQ.close(context)

And the client:

.. code-block:: julia
   :dedent: 1

    #!/usr/bin/env julia

    #
    # Hello World client in Julia
    # Connects REQ socket to tcp://localhost:5555
    # Sends "Hello" to server, expects "World" back
    #

    using ZMQ

    context = Context()

    # Socket to talk to server
    println("Connecting to hello world server...")
    socket = Socket(context, REQ)
    ZMQ.connect(socket, "tcp://localhost:5555")

    for request in 1:10
        println("Sending request $request ...")
        ZMQ.send(socket, "Hello")

        # Get the reply.
        message = String(ZMQ.recv(socket))
        println("Received reply $request [ $message ]")
    end

    # Making a clean exit.
    ZMQ.close(socket)
    ZMQ.close(context)


Copy these two programs to two files, server.jl and client.jl.  In different shells (separate terminals run them (enter on the command line) using:

.. code::

    $ julia server.jl


.. code::

   $ julia client.jl


Note that control-c will kill the server process.  If you change the permissions by setting the execute bit, you can run the file.   We will go line by line through the code to understand how this works. To bring in the ZeroMQ library:

.. code::

    import ZMQ

Each process needs a container for the sockets. This container is called a context:

.. code::

   context = zmq.Context()

We can create a socket in the context:

.. code::

   socket = ZMQ.Socket(ZMQ.REP)

A socket is a communication conduit. This is a request-reply type of socket indicated by 'REP'.   We need to select the communication protocol (tcp), label the address and select the port (5555):

.. code::

   ZMQ.bind(socket,"tcp://*:5678")

Note that there is nothing special about 5678.  We hope that it is high enough not to overlap ports used by other programs.
To receive a message:

.. code::

   message = ZMQ.recv(socket)


To send a message:

.. code::

   ZMQ.send(socket, "Hello")


The string function converts byte code to unicode.

.. code::

   message = String(ZMQ.recv(socket))

In Julia (and Python) strings are stored using Unicode. ZeroMQ uses byte strings, not Unicode. In Julia, the the ZMQ send command will take a unicode string and the conversion to a byte string is done automatically.  The receive commend returns a byte string which can be converted using the String function.

We take the previous example and code up a client server example where the client sends 10 (x,y) pairs to a server which computes the Two Link Manipulator Inverse Kinematics for each.






.. code-block:: julia
   :dedent: 1

    #!/usr/bin/env julia

    #
    #   Weather update server
    #   Binds PUB socket to tcp://*:5556
    #   Publishes random weather updates
    #

    using ZMQ

    context = Context()
    socket = Socket(context, PUB)
    bind(socket, "tcp://*:5556")

    while true
        zipcode = rand(10000:99999)
        temperature = rand(-80:135)
        relhumidity = rand(10:60)
        message = "$zipcode $temperature $relhumidity"
        send(socket, message)
        yield()
    end

    close(socket)
    close(context)



.. code-block:: julia
   :dedent: 1

    #!/usr/bin/env julia

    #
    #   Weather update client
    #   Connects SUB socket to tcp://localhost:5556
    #   Collects weather updates and finds avg temp in zipcode
    #

    using ZMQ

    context = Context()
    socket = Socket(context, SUB)

    println("Collecting updates from weather server...")
    connect(socket, "tcp://localhost:5556")

    # Subscribe to zipcode, default is NYC, 10001
    zip_filter = length(ARGS) > 0 ? int(ARGS[1]) : 10001

    subscribe(socket, string(zip_filter))

    # Process 5 updates
    update_nbr = 5

    total_temp = 0
    for update in 1:update_nbr
        global total_temp
        message = unsafe_string(recv(socket))
        zipcode, temperature, relhumidity = split(message)
        total_temp += parse(Int, temperature)
    end

    avg_temp = total_temp / update_nbr

    println("Average temperature for zipcode $zip_filter was $(avg_temp)F")

    # Making a clean exit.
    close(socket)
    close(context)






.. code-block:: julia
   :dedent: 1

    import ZMQ

    pub = ZMQ.Socket(ZMQ.PUB)
    ZMQ.bind(pub, "tcp://*:5678")


    for i in 1:20
        #  Do some 'work'
        sleep(1)
        println(i)
        #  Send to client
        ZMQ.send(pub,"Hello")
    end

    ZMQ.send(pub,"0")





.. code-block:: julia
   :dedent: 1

    import ZMQ


    #  Socket to talk to server
    println("Connecting to hello world server…")
    sub = ZMQ.Socket(ZMQ.SUB)
    ZMQ.subscribe(sub)
    ZMQ.connect(sub, "tcp://localhost:5678")


    #  Do 10 requests, waiting each time for a response
    while true
        #  Get the reply.
        message = String(ZMQ.recv(sub))
        println(message)
        if message == "0"
            break
        end
    end
