#!/usr/bin/python           # This is client.py file

import socket               # Import socket module

s = socket.socket()         # Create a socket object
host = "localhost" # Get local machine name
port = 12345                # Reserve a port for your service.

s.connect((host, port))
s.send("$RXHRB,301118,151258,0.00000,N,0.00000,E,NCTUW,1,2*2D")
s.close()                     # Close the socket when done