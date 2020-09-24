import math

x_off = 1.0702 
y_off = 9.86115

theta = math.pi / 4 #+ math.pi / 2

#-1.0702 -9.86115

with open("amcl.dat", "r") as f:
    data = f.readlines()
    out = open("amcl_mod.dat", "w")
    step = 5
    out.write('{}\n'.format((len(data) - 1)/ step))

    for line in data[1::step]:
        words = line.split()
        
        x = float(words[0])
        y = float(words[1])

        x = x + x_off
        y = y + y_off

        x_new = (x * math.cos(theta)) + (y * math.sin(theta));
        y_new = (-x * math.sin(theta)) + (y * math.cos(theta));
        
        out.write('{} {}\n'.format(x_new, y_new))

    out.close()