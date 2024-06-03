import mujoco as mj
from mujoco.glfw import glfw
import matplotlib.pyplot as plt
import numpy as np
import os
import queue
import control
import threading
import tkinter as tk
import tkinter.ttk as ttk
import rtbWrapper as rtb
from task import taskRequest as taskReq

# commmon method
# 
# check if the string s is a number or not.
# Note that the string including decimals returns true. 
# 
def isNum(s): 
    try:
        float(s)
    except ValueError:
        return False
    else:
        return True

def transpose(list_org): # doubly list e.g. [[0, 1, 2], [3, 4, 5], [6, 7, 8]]
    list_transposed = [list(x) for x in zip(*list_org)]
    return list_transposed

class RingBuffer:
    def __init__(self, size):
        self.buffer = [None for i in range(0, size)]
        self.top = 0
        self.bottom = 0
        self.size = size
        self.isFull = False

    def __len__(self):
        return self.bottom - self.top

    def add(self, value):
        self.buffer[self.bottom] = value
        self.bottom = (self.bottom + 1) % len(self.buffer)
        if(self.top == self.bottom):
            self.isFull = True

    def getVal(self, index=None):
        if index is not None:
            return self.buffer[index]

        value = self.buffer[self.top]
        self.top =(self.top + 1) % len(self.buffer)
        return value
    
    def getList(self):
        l = []
        if(self.isFull): # todo test
            l = self.buffer[self.bottom:]
            l.extend(self.buffer[:self.bottom])
        else:
            l = self.buffer[:self.bottom]
        return l