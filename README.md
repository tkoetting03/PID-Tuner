# PID-Tuner

## The Idea

I first began thinking about this small project when I was taking my Embedded Systems course and we were tasked with controlling a servo motor with an Arduino board. I began to look into it and found out about PID and plant controllers and began to research them. I made this program as a way to cap off my interest in this field. 

## The Mathematics

I assumed that the mathematics we would use here would need to be discrete and not continuous out of the assumption that I would be programming this with an MCU implementation in mind so I would need to be somewhat resource conscious (although I use a simulation to run it but we'll get to that later). I first began by finding the continuous math in order to find its discrete counterpart. 

[PLANT DIAGRAM]

Our diagram of this system consisters of our PID controller, our "Plant" or our simulation, and our reference position which we want the plant to reach. The controller has three components which sum into its output to correct the plant position if there is any error. 

$u(t)=K_p\cdot e(t)+K_i \int^t_0 e(t) dt+K_d\cdot \frac{de(t)}{dt}$

Our weights which we can alter are $K_p$ (proportional), $K_i$ (integral), and $K_d$ (derivative). Our proportional weight is a reaction to the value of the error, almost a 1:1 reaction. Our integral weight is a reaction to the total sum of the area under the error curve, so the more area accumulates the more it will try and correct the output. Our derivative weight is a reaction to the change in the error, if a sudden stop or start is detected this high rate of change will cause a significant reaction from the derivative weight. Each weight balances the other out, but sometimes it can be beneficial to only use two at a time (or even one depending on the situation). 

We now must find a way to make the integral and derivative components discrete so we can implement them into the code. 

### Integral Component

We can numerically approximate our integral using the trapezoid rule. The trapezoid rule works as follows:

We take an integral, say $\int^b_a f(x) dx$, we can find an equivalent value to $\int^b_a f(x) dx$ by taking:

$\int^b_a f(x) \ dx \approx \frac{b-a}{2}\big(f(a)+f(b)\big)$

Where $\frac{f(a)+f(b)}{2}$ is the average height and $b-a$ is width. If we take this as a discrete function we can find that

$\int^z_a f(x) \ dx \approx h \big[ \frac{1}{2} f(a) + f(b) + f(c) + f(d) + \dots \frac{1}{2} f(z)\big]$

Where $h$ is the uniform spacing between the samples. What we are concerned with most though is taking the change in the area from the previous measurement and adding (or subtracting if negative) it to the current area value. Thus we can defined a variable $\Delta I$ as:

$\Delta I \approx \frac{T_s}{2}\big(e[k]+e[k-1]\big)$

Where $e[k]$ is the error value and $e[k-1]$ is the previous error value, so that when subtracted and then divided by $2$ they form the height. $T_s$ (the sample time or our equivalent to $h$) is the width. We then multiply this result by our weight $K_i$ and add it to the previous integral area value to get our new area value:

$I[k] = I[k-1] + \Delta I \approx I[k-1] + K_i\cdot\frac{T_s}{2}\big(e[k]+e[k-1]\big)$

We implement this in the code as: 

```
    pid->integrator += 0.5 * pid->Ki * pid->Ts * (error + pid->prev_error);
```

Where ```pid->integrator``` is where we store our accumulated area of the integral in our corresponding PID struct. 

### Derivative Component

Now we move on to a discrete version of the derivative component. Obviously we cannot write an infinitely continuous derivative in our code if we assume resource limitations so we must find a discrete way to go about it. Finding this method is simpler than our integral component because the derivative is change with respect to time, so we can write the change between two points (or measurements) with respect to a set time period as

$\frac{y[k]-y[k-1]}{T_s}$

Where $y[k]$ is our current measurement, $y[k-1]$ is our previous measurement, and $T_s$ is our sample time. 

We can implement this in the code as:

```
  double derivative = (measurement - pid->prev_measurement) / pid->Ts;
```

We then multiply this by our weight $K_d$ to get:

$D[k]=K_p\cdot \frac{y[k]-y[k-1]}{T_s}$

But there is one last thing; when we take the derivative of error, $\frac{de(t)}{dt}=\frac{dr(t)}{dt}-\frac{dy(t)}{dt}$, and we eliminate the derivative of the setpoint since it is a constant and does not change, we get: 

$\frac{de(t)}{dt}=-\frac{dy(t)}{dt}$

This tells us that the derivative of the output should be inversely proportional to the error (error goes down when the output increases to match the set point). Thus we will change out equation for $D[k]$ to be negative: 

$D[k]=-K_p\cdot \frac{y[k]-y[k-1]}{T_s}$

### Simulating with a First Order Differential Equation

I personally do not have an Arduino connected servo or plant system to test this on, so I decided to simulate a semi-realistic feedback using a differential equation. We first set out by using the following equation:

$\frac{dy}{dt}=\frac{u-y}{\tau}$

Where $y$ is our plant output, $u$ is our controller output, and $\tau$ is our time constant. This equation is a basic way of modeling the initial rapid response of a system and the then gradual slowing of progress as it gets farther away from the initial command (and hopefully closer to the setpoint).



