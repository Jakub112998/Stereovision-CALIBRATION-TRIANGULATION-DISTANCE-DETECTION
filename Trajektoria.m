function plot3Ddata(x,y,z)
test = xlsread('D:\dokumenty\pulpit\Pomiar3.xlsx', 'W101:Y109');
x=test(:,1);
y=test(:,2);
z = test(:,3);
plot3(x,y,z)
xlabel('X')
ylabel('Y')
zlabel('Z')
grid on
end