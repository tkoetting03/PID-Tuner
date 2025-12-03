# PID-Tuner in C

## The Idea

I first began thinking about this small project when I was taking my Embedded Systems course and we were tasked with controlling a servo motor with an Arduino board. I began to look into it and found out about PID and plant controllers and began to research them. I made this program as a way to cap off my interest in this field. 

## The Mathematics

I assumed that the mathematics we would use here would need to be discrete and not continuous out of the assumption that I would be programming this with an MCU implementation in mind so I would need to be somewhat resource conscious (although I use a simulation to run it but we'll get to that later). I first began by finding the continuous math in order to find its discrete counterpart. 

<img src="/assets/system.png" alt="System Diagram" title="System Diagram" width="30%">

Our diagram of this system consisters of our PID controller, our "Plant" or our simulation, and our reference position which we want the plant to reach. The controller has three components which sum into its output to correct the plant position if there is any error. 

$u(t)=K_p\cdot e(t)+K_i \int^t_0 e(t) dt+K_d\cdot \frac{de(t)}{dt}$

Our weights which we can alter are $K_p$ (proportional), $K_i$ (integral), and $K_d$ (derivative). Our proportional weight is a reaction to the value of the error, almost a 1:1 reaction. Our integral weight is a reaction to the total sum of the area under the error curve, so the more area accumulates the more it will try and correct the output. Our derivative weight is a reaction to the change in the error, if a sudden stop or start is detected this high rate of change will cause a significant reaction from the derivative weight. Each weight balances the other out, but sometimes it can be beneficial to only use two at a time (or even one depending on the situation). 

### Error

We calculate our error using the following equation:

$e[k]=r[k]-y[k]$

Where $e[k]$ is the error at our step $k$, $r[k]$ is our setpoint, and $y[k]$ is our measurement at step $k$. 

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

Where $y$ is our plant output, $u$ is our controller output, and $\tau$ is our time constant. This equation is a basic way of modeling the initial rapid response of a system and the then gradual slowing of progress as it gets farther away from the initial command (and hopefully closer to the setpoint). Solving for this differential equation we get a exponential decay function of:

$y(t)=1-e^{-t}$

And is shown in the graph below: 

<img src="/assets/graph.png" alt="Exp. Decay Graph" title="Exp. Decay Graph" width="60%">

As long as we keep $\tau$ the same value, no matter what our value of $u$ is it will still take the same amount of time to reach $\frac{dy}{dt}=0$ (which is technically infinity but I digress). 

So essentially the differential equation is modeling how the physical system's output changes over time when you push on it with an input. 

## The Programming

First things first we will be making a PID object with all the variables we outlined above for our PID Controller. We will hold this in our pid.h header file:

```
typedef struct {
    double Kp;
    double Ki;
    double Kd;

    double Ts;

    double integrator;
    double prev_error;
    double prev_measurement;

    double out_min;
    double out_max;
} PID;
```

Kp, Ki, and Kd are our component weights $K_p, \ K_i, \ K_d$, and Ts is our sample time $T_s$. The double variable integrator is the variable in which we will hold the area under the curve calaculated by our integral component, our prev_error variable stores the error value of the previous step, and likewise the prev_measurement variable stores the measurement value from the previous step. Finally we have out_min and out_max, two variables which we did not outline in the Mathematics section and will get to later, their core purpose is essentially to act as boundaries for how high or low some specified value can be. 

Now moving to our primary pid.c file, we begin by simply creating our init function that initializes all the variables to provided values (whatever those may be, we will deal with such things later) and all remaining values are set to 0 since they are not user defined: 

```

void PID_Init(PID *pid,
              double Kp,
              double Ki,
              double Kd,
              double Ts,
              double out_min,
              double out_max)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;

    pid->Ts = Ts;

    pid->integrator = 0.0;
    pid->prev_error = 0.0;
    pid->prev_measurement = 0.0;

    pid->out_min = out_min;
    pid->out_max = out_max;
}
```

We will make a function which copies the init functions assignment of the component weights $K_p, \ K_i, \ K_d$ in order to have a function we can call if we just want to change the weights. We will do the same for the output limits, taking the code from the init function again and creating a separate function in which we can separately declare the minimum and maximum for the provided variables. We would add a function call in the function to set the output limits but before write this we need to define our function which accomplishes this. We will call this function our clamp function:

```
static double clamp(double x, double min_val, double max_val)
{
    if (x > max_val) return max_val;
    if (x < min_val) return min_val;
    return x;
}
```

Our main purpose for this function will to throttle any drastic changes in the output value, and to also keep the integrator value from getting to large in either direction. We simply check if the value is larger or smaller than our provided maximum and minimum and if it is we return the maximum or minimum value. If this is not the case then the passed value is just returned unaltered.

Now returning to our function where set the minimums and maximums, after setting our struct variables to the provided values, we will then call the clamp function to check if the integrator value exceeds the new boundaries:

```
pid->integrator = clamp(pid->integrator, out_min, out_max);
```

Then we have another separate derivative function which sets the integrator, prev_error, and prev_measurement variables to 0, functionally resetting them. I have not shown any of the functions derivative of the init function because the code is literally an exact copy of the contents of the init function so showing it again would be redundant. 

We now move on to the most important function of the program is the function which updates the PID for each step taken. We outlined most of the code for the math function in the Mathematics section, and there isn't much more required than that. We first calculate error:

```
double error = setpoint - measurement;
```

Then using our previous code for the integrator function we have


```
    pid->integrator += 0.5 * pid->Ki * pid->Ts * (error + pid->prev_error);
    pid->integrator = clamp(pid->integrator, pid->out_min, pid->out_max);
```

Where we clamp the integrator value to ensure it doesn't endlessly grow in either direction. We then calculate the derivative:

```
double derivative = (measurement - pid->prev_measurement) / pid->Ts;
```

Now we return to our equation for the controller output $u(t)$, which is:

$u(t)=K_p\cdot e(t)+K_i \int^t_0 e(t) dt+K_d\cdot \frac{de(t)}{dt}$

Writing this in our program we have:

```
    double output = pid->Kp * error + pid->integrator - pid->Kd * derivative;
```

We then put boundaries on our output using the clamp function:

```
    output = clamp(output, pid->out_min, pid->out_max);
```

And finally we set our struct variables prev_error and prev_measurement to the measurement and error values at the current step $k$ before returning the output. 

```
    pid->prev_error = error;
    pid->prev_measurement = measurement;

    return output;
```

## Main Function

We now have our code figured out for our PID controller, and now we need to implement a main function capable of interacting and simulating a system response. I outlined in the Mathematics section how we would accomplish simulating a system response by using a first order differential equation; $\frac{dy}{dt}=\frac{u-y}{\tau}$. Implementing this in our function which simulates the steps of the plant we have:

```
static double plant_step(double y, double u, double Ts, double tau)
{
    double dy = (-y + u) / tau;
    return y + Ts * dy;
}
```

Where the first equation is the function describing $\frac{dy}{dt}$. We then use Euler integration to step the differential equation forward (take a step $T_s$ in size using $\frac{dy}{dt}$) in time by our set amount $T_s$, so we return our new output value $y_{new}$ and add $T_s\cdot \frac{dy}{dt}$ to $y_{old}$ to get $y_{new}$.

The rest of our main function is mostly just us setting up our variables to pass to pid.c and stepping the plant. 

```
    PID pid;
    double Ts = 0.01;
    double out_min = -10.0;
    double out_max =  10.0;

    PID_Init(&pid, 1.0, 0.0, 0.0, Ts, out_min, out_max);

    double tau = 0.5;
    double setpoint = 1.0;
```

We then create an infinite while loop to endlessly loop while executing plant steps and taking input for $K_d$, $K_i$, and $K_p$. After setting all values for the weights and resetting the PID struct values, we declare the plant output and the time, and begin a for loop to iterate for 4 seconds every tenth of a second, update the PID controller, step the differential equation, and print the resulting time, goal setpoint, differential equation value at that step, and the PID controller output. Then we increment t to keep track of what step we're on and loop through the for loop again. 

```

    for (int k = 0; k < 400; ++k) { 
            double u = PID_Update(&pid, setpoint, y);
            y = plant_step(y, u, Ts, tau);

            printf("%.3f\t%.3f\t%.3f\t%.3f\n", t, setpoint, y, u);

            t += Ts;
        }

```



