import math

sigma_sq = 90
kernel = ""
kernel_size = 20
for i in range(-kernel_size,kernel_size+1):
    for j in range(-kernel_size,kernel_size+1):
        dist = (i*i)+(j*j)
        kernel = kernel + "{:5.4f}".format(math.exp(-.5*dist/sigma_sq)) + " "
#        kernel = kernel + "{:5.4f}".format(1/dist) + " "
    kernel = kernel + "\n"

print(kernel)
