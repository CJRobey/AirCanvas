import socket
import binascii
import numpy as np
import time
import random
import uuid

class dust:
    global s
    def __init__(self, TCP_IP = '127.0.0.1'):
        #TCP_IP = '127.0.0.1'
        TCP_PORT = 53309
        self.BUFFER_SIZE = 4096
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((TCP_IP, TCP_PORT))
        TCP_PORT = 53309
        BUFFER_SIZE = 4096
    def start(self):
        global s
        TCP_IP = '127.0.0.1'
        TCP_PORT = 53309
        BUFFER_SIZE = 4096
        s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        s.connect((TCP_IP, TCP_PORT))
        TCP_PORT = 53309
        BUFFER_SIZE = 4096
        return s

    def close(self):
        self.s.close()

    def genId(self):
        return str(("{" + str(uuid.uuid4()) + "}").encode('utf8'))[2:-1]
    def add_a_node(self,new_node,old_node,x,y,z,r=.01,):
        #global node_count
        return self.s.send(binascii.hexlify("addnodewithid ".encode('utf8') + str(new_node).encode('utf8') +" ".encode('utf8')+str(x).encode('utf8') + " ".encode('utf8') +str(y).encode('utf8') + " ".encode('utf8') + str(z).encode('utf8') + " ".encode('utf8') +str(r).encode('utf8') + " ".encode('utf8') + str(old_node).encode('utf8') +" ".encode('utf8'))+ "\0".encode('utf8'))
    def add_n_node(self,new_node,x,y,z, r=.01):
        #global node_count
        return self.s.send(binascii.hexlify("addnodewithid ".encode('utf8') + str(new_node).encode('utf8') +" ".encode('utf8')+str(x).encode('utf8') + " ".encode('utf8') +str(y).encode('utf8') + " ".encode('utf8') + str(z).encode('utf8') + " ".encode('utf8') +str(r).encode('utf8'))+ "\0".encode('utf8'))

        #node_count += 1
        #return node_count -1
    def add_edge(self,node_0, node_1):
        return self.s.send(binascii.hexlify("addedge ".encode('utf8') +str(node_0).encode('utf8')+" ".encode('utf8')+str(node_1).encode('utf8')+"".encode('utf8')) + "\0".encode('utf8'))
    def getSnapShot(self):
        return s.send(binascii.hexlify("getSnapShot".encode('utf8'))+ "\0".encode('utf8'))
    def exportAsObj(self, write = 0, fname = "model.obj"):
        self.s.send(binascii.hexlify("exportAsObj".encode('utf8'))+ "\0".encode('utf8'))
        count = 0
        while(write and count <5):
            reply_ = self.reply()
            if(reply_ != -1):
                f = open(fname, "w+")
                f.write((reply_))
                f.close()
                print(reply_) 
                return 1
            count = count +1
            self.s.send(binascii.hexlify("exportAsObj".encode('utf8'))+ "\0".encode('utf8'))
           # return reply_
        #return self.reply()
        return 1

    def clear(self):
            return self.s.send(binascii.hexlify("new".encode('utf8'))+ "\0".encode('utf8'))
    #reply_ = bytes()
    def reply(self):
        #global reply_
        response = bytes()
        while True:
            oneEnd = response.find(chr(0).encode('utf8'))
            if (-1 == oneEnd):
                response += self.s.recv(self.BUFFER_SIZE)
                continue
            reply_ = response[:oneEnd]
            response = response[oneEnd + 1:]
            output  =binascii.unhexlify(reply_)
            print(output[:14])
            if(output[:12] == "#exportready".encode('utf8')):
                print("test EXPORT")
                self.exportAsObj(0)
            #print (((reply_))[:14])
            if(((reply_))[:14] == b'2b4f4b0d0a2320'):
                f = open("model.obj", "w+")
                f.write((binascii.unhexlify(reply_))[14:].decode('utf8'))
                f.close()
                
        else:
            return -1


