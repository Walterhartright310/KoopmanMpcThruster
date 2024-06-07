#注意 SX MX 的区别
from casadi import *
x = SX.sym('x',2 )
print(x)
y=SX.sym('y')
f=Function('f',[x,y],[x,sin(y)*x],['x','y'],['r','q'])
print("f function is:",f)

r1,q1=f([1.0,3.0],2)

print('r1:',r1)
print('q1:',q1)

#y = MX.sym("y"，5 )

