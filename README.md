# CarND-Controls-MPC
Self-Driving Car Engineer Nanodegree Program


# Compilation
## Your code should compile.
Yes, it compiles.

# Implementation
## The Model
The model kinematic expresions are the one viewed on the lesson.

     fg[2+x_start   +i]= x1 - (x0 + v0*CppAD::cos(psi0) * dt);
     fg[2+y_start   +i]= y1 - (y0 + v0*CppAD::sin(psi0) * dt);
     fg[2+psi_start +i]= psi1 - (psi0 - v0*delta0/Lf * dt);
     fg[2+v_start   +i]= v1 -(v0 + a0 * dt);
     fg[2+cte_start +i]=cte1 - ( (f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
     fg[2+epsi_start+i]=epsi1 - ((psi0 - psides0) - v0*delta0/Lf * dt);   
     
 The main problem I had with this project was a typo error in
 
     fg[2+cte_start +i]=cte1 - ( (f0 - y0) + (v0 * CppAD::sin(epsi0) * dt));
     
where I had a operator error: (f0 - y0) + (v0 - CppAD::sin(epsi0) * dt, change * by -. And all was bad.

The main dificulty with this project is that it is very dificult to debug, and a minimal operator o sign error make all work wrong with any compilation error or idea where it is.

## Timestep Length and Elapsed Duration (N & dt)
As lower dt and bigger N, the better result should be. But this is at the price of power computation. 

I choosed N=10 and dt=0.1, a bigger value for N don't five a neglectable improvement, since I just keep the first prediction. 

I think in a real car dt would be 0.03 or lower in order to have a better time response. But it is not needed on for the simulator case.

## Polynomial Fitting and MPC Preprocessing
Using the Self-Driving Car Project Q&A | MPC Controller suggestion, I changed coordinates from global to vehicle local:

          for(unsigned int i=0;i<ptsx.size();i++){
            double shift_x=ptsx[i]-px;
            double shift_y=ptsy[i]-py;
            ptsx[i]=(shift_x*cos(0-psi)-shift_y*sin(0-psi));
            ptsy[i]=(shift_x*sin(0-psi)+shift_y*cos(0-psi));
          }



## Model Predictive Control with Latency
The point is add to the state predicted the delay based upon the actual values os steer and throttle.

	  // adding delay
	  const double delay_t = 0.1;
	  const double Lf = 2.67;
          // based upon t-1 steer and throttle values
          steer_value = j[1]["steering_angle"];
          throttle_value = j[1]["throttle"];

	  // embed the actuator delay into new state vector simulating the delay
	  double delayed_x = v * delay_t;
	  double delayed_y = 0;
	  double delayed_psi = - v * steer_value / Lf * delay_t;
	  double delayed_v = v + throttle_value * delay_t;
	  double delayed_cte = cte + v * sin(epsi) * delay_t;
	  double delayed_epsi = epsi - v * steer_value / Lf * delay_t;



# Simulation
## The vehicle must successfully drive a lap around the track.
The vehicle drives properly.	
