.. _`appendixA`:

Appendix
==================

Robotics Accident Data
----------------------

.. _`tab:deathstats`:
.. table::  Table was compiled from US Government office OSHA, 30 years, 1987-2017, of OSHA Robotics related fatal accident reports.
   :align:  center
   :widths: auto

   +------------+-----------------------------------------------------------+
   | **Date**   | **Description of fatality in OSHA report**                |
   +============+===========================================================+
   | 6/29/87    | Employee Crushed And Killed By Robot Arm                  |
   +------------+-----------------------------------------------------------+
   | 11/28/87   | Employee Crushed To Death While Working On A Lathe        |
   +------------+-----------------------------------------------------------+
   | 5/17/89    | Employee Crushed And Killed By Industrial Robot           |
   +------------+-----------------------------------------------------------+
   | 10/1/92    | Employee Killed When Crushed By Robot Arm                 |
   +------------+-----------------------------------------------------------+
   | 3/13/93    | Employee Killed When Crushed By Robot                     |
   +------------+-----------------------------------------------------------+
   | 4/8/94     | Employee Dies From Coronary Insufficiency                 |
   +------------+-----------------------------------------------------------+
   | 9/27/94    | Employee Crushed To Death By Activated Robot              |
   +------------+-----------------------------------------------------------+
   | 2/15/96    | Employee Killed When Burned By Robotic Hot Metal Pourer   |
   +------------+-----------------------------------------------------------+
   | 1/27/97    | Crushed By Robot Arm During Machine Cycle                 |
   +------------+-----------------------------------------------------------+
   | 4/29/97    | Employee Struck By Material Handling Equipment            |
   +------------+-----------------------------------------------------------+
   | 5/4/99     | Employee Killed When Crushed By Robot Arms                |
   +------------+-----------------------------------------------------------+
   | 6/8/99     | Employee Killed When Crushed By Robot Against Conveyor    |
   +------------+-----------------------------------------------------------+
   | 8/27/99    | Employee Killed In Robotic Weld Cell                      |
   +------------+-----------------------------------------------------------+
   | 12/29/01   | Employee Killed When Robot Pinned His Neck                |
   +------------+-----------------------------------------------------------+
   | 7/28/03    | Employee Is Killed When Crushed By Equipment              |
   +------------+-----------------------------------------------------------+
   | 12/13/03   | Employee Is Killed When Caught In Robotic Arm             |
   +------------+-----------------------------------------------------------+
   | 3/30/04    | Employee Was Killed By Industrial Robots                  |
   +------------+-----------------------------------------------------------+
   | 3/22/06    | Employee Dies When Struck By Robotic Equipment            |
   +------------+-----------------------------------------------------------+
   | 7/24/06    | Employee Is Killed When Crushed By Robot                  |
   +------------+-----------------------------------------------------------+
   | 10/31/06   | Worker Is Killed, Lockout Procedures Not Followed         |
   +------------+-----------------------------------------------------------+
   | 5/13/07    | Employee Dies After Being Struck By Robotic Arm           |
   +------------+-----------------------------------------------------------+
   | 7/21/09    | Employee Is Killed By Robotic Palletizer                  |
   +------------+-----------------------------------------------------------+
   | 8/2/11     | Employee Is Killed When Caught In Equipment               |
   +------------+-----------------------------------------------------------+
   | 12/15/12   | Robot Crushes And Kills Worker Inside Robot Work Cell     |
   +------------+-----------------------------------------------------------+
   | 3/7/13     | Maintenance Worker Is Struck And Killed By Robot          |
   +------------+-----------------------------------------------------------+
   | 6/16/13    | Employee Is Struck By Axis Arm, Later Dies                |
   +------------+-----------------------------------------------------------+
   | 7/7/15     | Employee Is Killed When Head Is Crushed By Robot Arm      |
   +------------+-----------------------------------------------------------+
   | 6/19/16    | Employee is Killed While Making Repairs                   |
   +------------+-----------------------------------------------------------+



Linux Command Environment
-------------------------

We will survey basic operations which are common to all Linux systems.
For the casual user of the command line, Linux, Unix and even OSX
systems are the same.

Linux is an operating system, i.e., a language by which you communicate
with the computer. Linux is one the most popular true operating systems.
This means you are likely to encounter it in your own environment. Linux
is written in C/C++, and so not essential, knowledge of C/C++ is very
useful when working on all the variants of Linux.

Windowing
~~~~~~~~~

The graphical display system on top of Linux is referred to as X Windows
(although it should be called the X windowing system). The Ubuntu is
currently running the X window system with the Gnome window manager.
Type Ctrl + Alt + T to bring up a terminal.

The more commonly used commands are

ls
    This command stands for list. It asks that the contents of the
    directory be listed. It is similar to the dos command dir. In the
    terminal:

    ::

        > ls

cp
    This is the copy file command. So if you have a file named foo.c and
    you want a copy, say as a backup, you would just type

    ::

        > cp foo.c foo.backup

mv
    This command stands for move. The primary use is to rename files.
    For example, if you have a file named foo and you want to call it
    foo.bar the you would type

    ::

        > mv foo foo.bar

rm
    This command stands for remove. It is the delete command. Usage is
    normally

    ::

        > rm foo.bar

mkdir
    This command stands for make directory (a directory is a folder).
    From your current directory (if you are in your login directory),
    you can create a subdirectory named foodir by

    ::

        > mkdir foodir

rmdir
    This command stands for remove directory. You can remove a
    subdirectory named foodir by

    ::

        > rmdir foodir

    The directory must be empty to do this.

cd
    This command stands for change directory. You would use this command
    to connect to the directory you just created above by

    ::

        > cd foodir

Test your knowledge by creating a robotics directory.

Creating a file
~~~~~~~~~~~~~~~

Using gedit, you can create a simple program. Type the following into
the editor:

::

    #include <stdio.h>
    int main()
    {
       printf("Hello World\n");
       return 0;
    }

Save this as hello.c: click File and then Save, navigate to your
robotics directory, but do not exit.

Compiling
~~~~~~~~~~

To compile the Hello file, go back to your terminal window and type

::

    > cd robotics
    > gcc -o hello  hello.c

(note that this assumed you created the robotics directory.) To get used
to how things work, make some errors in the hello.c file so that you can
see what messages you get.

If you made errors, you should see error messages displayed. Fix these
errors and recompile. Don’t forget to save your program after correcting
the errors. To recompile, go back to the shell window and hit the up
arrow key. This should bring back the previous command so you don’t have
to retype it. You can hit the up arrow key repeatedly to bring back past
commands. Also, !g will run the last command starting with the letter
’g’. If your program compiles correctly, nothing is printed by the
compiler.

There is one odd behaviour that you should know about. If your program
uses math.h and you want to compile this on a Linux/Unix system you need
to use:

::

    > gcc -o hello  hello.c -lm

The C language (and Unix) began before math coprocessors were available.
Math functions were done in software and they took significant space.
The decision was made to not automatically include them to save space
and compile time. And so you needed to add the -lm to the compile (gcc)
line. At this point, all of this is history, but convention has not
changed.

To run any executable (program) under Linux, simply type its name (and
any required command-line arguments) at the command prompt in a terminal
window. If Linux complains that it cannot find the file, and you see it
in the directory listing, try typing:

::

     ./filename

To run your program, just enter the name of the executable as a shell
command. In this case, just type hello and hit enter. With the pcDuino
as installed, the system will likely say that there is no such command.
This is because the current directory, referred to as . (just one dot),
is not in your path by default. The easy way to run hello right now is
to type the command ./hello which specifies to run the file hello in the
current directory.

Note: if you don’t like putting “./” in front of your command, you can
have the system do this. To put the current directory in your default
path, do the following:

-  In Leafpad, click file, open, and select ubuntu from the list on the
   left.

-  In the upper left is what looks like a pencil. It should say “Type a
   file name” when you cursor over it. Click on that.

-  Type in the file name .bashrc (including the dot) in the box marked
   “Location”

-  Hit return (or click open in the lower right)

-  The file has a lot of stuff in it you do not need to worry about.

-  After the three comment lines (starting with #) at the start of the
   file, insert the line

   ::

        export PATH=$PATH:.

-  Save the file and exit Leafpad. Note that if you did not make a
   change, “save” is gray, you cannot select save if you have not made
   any changes. This is true in general for Leafpad.

In general, to compile a C file, type

::

    gcc -Wall -g filename.c -o filename

The -Wall switch turns on all warnings, and the -g switch adds debugging
information. The o filename specifies the name of the resulting
executable (the default is a.out). If you get compiler errors, go back
to your editor and fix them. Save your file and recompile until you can
compile successfully.

If you type only gcc foo.c you will get a default name of a.out. If you
only want to compile a function, not link it with anything else, you
type

::

    gcc -c foo.c

This compiles the source to object code, foo.o. For programs with
several external functions (meaning multiple files), a longer sequence
is required. Say you have a main routine prog.c, and this calls external
functions, funct1.c, funct2.c, and requires the library (static linking)
lib.a. You would have to compile each function and the main:

::

    cc -c prog.c
    cc -c funct1.c
    cc -c funct2.c

Then you can link these together into an executable:

::

    gcc -o prog prog.o funct1.o funct2.o lib.a

| For further practice, try the following:
| Type “cd /ho<tab>” the shell should expand the tab so that you see
  “home/”. The shell will expand a tab to a file name if there is only
  one way to expand it or expand it until there is a difference. Try “cd
  /ho<tab>/ubu<tab>”. Do a cd to /usr/include to see the include files
  available. This can be handy if you are trying to find a function. For
  example, try

::

    grep random *.h | more

| This will show all the lines that have the word random in them. Use a
  space to tell the command more to display the next screen.
| Use the cd command with no parameters to get back to your home
  directory. Note that this command (cd) in DOS prints the current
  working directory like pwd in Linux, but cd in Linux takes you to your
  current working directory.

::

    cd                   Change back to home directory

**Redirection:** At times you may want to capture the output of a
command into a file. There is a redirection command available. For
example, if you want to list the contents of a directory and save this
in a file, type ls > foo and the file foo will be created and output of
the ls command entered. Another example is the intended use of the cat
command. This command will allow you to concatenate two files (glue them
together). If you have two files: file1 and file2 and want to connect
them into file3, then cat file1 file2 > file3. In a similar vein is the
append command. If you want to append the output of the ls command to
the end of file3, then ls >> file3.

Less commonly but still useful commands
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

cat
    This command stands for concatenate. It is often used to send the
    contents of a file display although this is not the main purpose of
    the command. cat foo will send the contents of the file foo to the
    display. See the section below on redirection on other uses of cat.

diff
    File compare. To compare two files, say file1 and file2, type diff
    file1 file2.

df
    Report the free and used disk (partition) blocks.

du
    Report disk usage for a directory or file system (use with the k
    option du -k).

ps
    Process status. This lists the processes you have from your current
    shell. The command line options here depend on which flavor of unix
    you run. Normally ps -aux will list out all of the processes on the
    machine.

kill
    This command will send a signal to a process. Normal usage is to
    terminate a process kill -9 pid where the pid is the process ID and
    is given by ps.

nice
    This command is used to change the priority of processes. For
    example, you wish to run a program called gnubeast. Say that this
    program attempts to disprove Fermat’s conjecture by finding a
    counter-example. You have a nagging feeling that this could run for
    a long long time and use lots of system resources. Then you should
    run the program at a lower priority so that other users are not
    adversely affected by your computations. This is done by nice
    gnubeast and this will lower the priority by 10 units (a system
    measure). You can be even nicer by typing nice +15 gnubeast.

grep
    Search a file for a pattern (think Get Regular ExPression). Usage is
    grep pattern file. Example: grep foo bar.c will print out the lines
    in the file bar.c which have the string foo.

tar
    Tape Archive. A utility to dump a directory or list of files into
    one file. Usage is tar -cf foo.tar foo where foo is a directory name
    or a list of files. To recover this, try tar -xf foo.tar.

gzip
    File compression. Usage is gzip foo and output will be foo.gz.

gunzip
    File uncompression. Usage is gunzip foo.gz.

ssh
    Remote secure shell. Will allow a command to be run on a remote
    computer. Usage is ssh linux101

scp
    Remote copy. Usage: scp linux101: /home/users/amy/foo.c . This will
    copy the file foo.c out of your remote home directory to your local
    directory.

Installation of Python and Gazebo
---------------------------------

Install the SciPy Stack: a collection of open source libraries for
scientific computing in Python

::

    sudo apt-get install python-numpy python-scipy python-matplotlib ipython
    ipython-notebook python-pandas python-sympy python-nose

Install the Latex typesetting language through TexLive (libraries) and
TexMaker (editor)

::

    sudo apt-get install texlive-full
    sudo apt-get install texmaker

| Install Gazebo (a robot simulator):
| Setup your computer to accept software from
  packages.osrfoundation.org.

::

    sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu
    `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-latest.list'

Setup keys

::

    wget http://packages.osrfoundation.org/gazebo.key -O - | sudo apt-key add -

Install Gazebo

::

    sudo apt-get update
    sudo apt-get install gazebo5-build-deps
    sudo apt-get install gazebo5
    sudo apt-get install libgazebo5-dev

Check your installation (initial execution takes extra time)

::

    gazebo

.. figure:: ../Terms/TermsFigures/twolinkexample2.*
   :width: 50%
   :align: center

   The Two Link Manipulator code results with a larger step in
   computation of the angle values. The servo movement will connect the
   vertices, but the path is not straight (you end up with something
   similar to a polygon, but with curved segments). The simulation will
   show a stop and go like behavior.

::

    import numpy as np
    import matplotlib.pyplot as plt
    import time
    from math import *

    a1 = 15
    a2 = 10
    step = np.pi/4
    N = 10

    t = np.arange(0, 2*np.pi+step, step)
    x = 5*np.cos(t) + 10
    y = 5*np.sin(t) + 8
    xsim1 = np.zeros((t.size-1)*N)
    ysim1 = np.zeros((t.size-1)*N)
    xsim = np.zeros((t.size-1)*N)
    ysim = np.zeros((t.size-1)*N)

    a1 = 15.0
    a2 = 10.0
    d = (x*x + y*y - a1*a1 - a2*a2)/(2*a1*a2)
    t2 = np.arctan2(-np.sqrt(1.0-d*d),d)
    t1 = np.arctan2(y,x) - np.arctan2(a2*np.sin(t2),a1+a2*np.cos(t2))

    for i in range(t.size-1):
      t1step = np.linspace(t1[i],t1[i+1], N)
      t2step = np.linspace(t2[i],t2[i+1], N)
      for k in range(N):
        xsim1[N*i+k] = a1*np.cos(t1step[k])
        ysim1[N*i+k] = a1*np.sin(t1step[k])
        xsim[N*i+k] = a2*np.cos(t1step[k]+t2step[k]) + xsim1[N*i+k]
        ysim[N*i+k] = a2*np.sin(t1step[k]+t2step[k]) + ysim1[N*i+k]

    #  Plot code removed for space
    #  Same as previous example (from Plots to Animation)
    for k in range((t.size-1)):
      for j in range(N):
          i = k*N+j
          x1 = xsim1[i]
          y1 = ysim1[i]
          x2 = xsim[i]
          y2 = ysim[i]
          plt.setp(arm,xdata = [0,x1,x2], ydata = [0,y1,y2])
          plt.draw()
          plt.plot([x2],[y2],'b.')
      time.sleep(0.15)

    plt.ioff()
    #plt.savefig("twolinkexample.pdf",format="pdf")  # if you want to save
    plt.show()

Mobile Example
~~~~~~~~~~~~~~

We provide a similar example to what was done in the Python Chapter.

.. math::

   \displaystyle \left(\frac{dx}{dt}, \frac{dy}{dt}\right) =
   \left\{
   \begin{array}{ll}
   (0.5t, 0.0),  & 0 \leq t < 2, \\[3mm]
   (0.25t, 0.65t),  & 2 \leq t < 5,
   \end{array}
   \right.

 and starting at :math:`t=0`, :math:`(x,y)  = (1,1)`.

::

    line, = plt.plot([],[],'bo')
    plt.xlim(0, 6)
    plt.ylim(0, 8)
    plt.xlabel('x')
    plt.ylabel('y')
    plt.draw()
    x = 1.0
    y = 1.0
    dt = 0.1

    for t in np.arange(0,5,dt):
        if t < 2:
            x = x + 0.5*t*dt
        if (t>=2):
            x = x + 0.25*t*dt
            y = y + 0.65*t*dt
        line.set_xdata([x])
        line.set_ydata([y])
        plt.draw()
        plt.plot([x],[y],'bo')

    plt.ioff()
    plt.show()

.. _`mobileexamplefig`:
.. figure:: ../Terms/TermsFigures/mobileexample.*
   :align: center
   :width: 85%

   A plot of the simple mobile example.


.. _`clientC`:

Client Code in C
-------------------------

The following code implements the client control code in C. It is a
minor modification of the free download at
http://www.linuxhowtos.org/data/6/client.c.

::

    #include <stdio.h>
    #include <stdlib.h>
    #include <unistd.h>
    #include <string.h>
    #include <sys/types.h>
    #include <sys/socket.h>
    #include <netinet/in.h>
    #include <netdb.h>

    void error(const char *msg)
    {
        perror(msg);
        exit(0);
    }

    int main(int argc, char *argv[])
    {
        int sockfd, portno, n;
        struct sockaddr_in serv_addr;
        struct hostent *server;

        char buffer[256];
        if (argc < 3) {
           fprintf(stderr,"usage %s hostname port\n", argv[0]);
           exit(0);
        }
        portno = atoi(argv[2]);
        sockfd = socket(AF_INET, SOCK_STREAM, 0);
        if (sockfd < 0)
            error("ERROR opening socket");
        server = gethostbyname(argv[1]);
        if (server == NULL) {
            fprintf(stderr,"ERROR, no such host\n");
            exit(0);
        }
        bzero((char *) &serv_addr, sizeof(serv_addr));
        serv_addr.sin_family = AF_INET;
        bcopy((char *)server->h_addr,
             (char *)&serv_addr.sin_addr.s_addr,
             server->h_length);
        serv_addr.sin_port = htons(portno);
        if (connect(sockfd,(struct sockaddr *) &serv_addr,sizeof(serv_addr)) < 0)
            error("ERROR connecting");

        while(1) {
         printf("> ");
         bzero(buffer,256);
         fgets(buffer,255,stdin);
         n = write(sockfd,buffer,strlen(buffer));
          if (n < 0)
            error("ERROR writing to socket");
          bzero(buffer,256);
              n = read(sockfd,buffer,255);
              if (n < 0)
                  error("ERROR reading from socket");
              printf("%s\n",buffer);
        }
        close(sockfd);
        return 0;
    }
