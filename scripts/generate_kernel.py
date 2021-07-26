import math

sigma_sq = 256
kernel = ""
kernel_size = 30
for i in range(-kernel_size,kernel_size):
    for j in range(-kernel_size,kernel_size):
        dist = (i*i)+(j*j)+1
        kernel = kernel + "{:5.4f}".format(math.exp(-.5*dist/sigma_sq)) + " "
#        kernel = kernel + "{:5.4f}".format(1/dist) + " "
    kernel = kernel + "\n"

print(kernel)
