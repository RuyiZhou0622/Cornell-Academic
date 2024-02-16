def iterate(ci, cr, max_iterations):

    iterations = 0

    zi = 0
    zr = 0

    for i in range(max_iterations):
        iterations += 1

        zr_temp = zr*zr - zi*zi + cr
        zi_temp = 2*zr*zi + ci

        zi = zi_temp
        zr = zr_temp

        if ((zi*zi + zr*zr) > 4):
            break
    print('The number of iteration is: ', iterations)
    return iterations

ci = 1.25
cr = 1.25
max_iterations =1000
result = iterate(ci, cr, max_iterations)
print(f'result = {result}')