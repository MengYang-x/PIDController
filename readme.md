### 基本概念
所有的PID都可以归为模拟PID和数字PID两大类。       
![](images/1.png)    
* 模拟PID通过搭建控制电路实现的
* 数字PID:通过编程实现
![](images/2.png)      
PID控制器在时间坐标下的连续表达式：   
![](images/3.png)     
PID控制器的离散表达式：   
![](images/4.png)     
公式里的△t是一个常量，因此可以跟比例系数Kp和微分系数Kd合并，可以得到位置式PID算法的公式：    
![](images/5.png)      
**当前时刻的位置式PID-上一时刻的PID，可以得到增量式PID的表达式：**    
![](images/6.png)     
![](images/8.png)  
![](images/7.png)    
位置式PID和增量式PID本质上是同一个公式。
#### 各项含义
![](images/3.png)
U = P + I + D    
1. P = Kp*(目标值-瞬时值)，单独调节无法达到目标值，而且会出现波动
2. I = Ki*[(目标瞬时值-目标值)+(上一秒的瞬时值-目标值)+...]*dt(时间间隔)
3. D = Kd*[(目标值瞬时值-目标值)+(上一秒的瞬时值-目标值)]/dt(时间间隔)

1. Kp:更快达到目标值，在只有积分控制下，**会出现转速波动和稳态误差这两个问题**。![](images/9.png)
2. Kd:相当于一个阻尼器，转速远离目标值时，阻止远离。转速靠近目标值时，阻止靠近。**可以用于消除转速波动问题**。![](images/10.png)
3. Ki:用于消除稳态误差，根据积分控制部分的公式，当实际值小于目标值时，对这部分偏差进行正向积分，在积分的作用下，积分控制的输出变得越来越大，因此输出也变得越来越大。当实际值大于目标值时，对偏差部分进行负向积分，减小输出。**可以用于消除稳态误差问题**。![](images/11.png)
想要实现的效果：快准稳。![](images/12.png)
#### ESP32 PWM输出
