#!/usr/bin/python
with open(“amcl.dat”, “r”) as f:
    data = f.readlines()
    
    for line in data:
    words = line.split()
    print words
