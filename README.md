# consensus.algoIC

### Rendezvous of 10 Integrating Agents
![alt text](https://github.com/marcotulio956/consensus.algoIC/blob/master/img/rzvs-1d_10agents.png)

### Rendezvous of 20 Integrating Agents with a Leader
![alt text](https://github.com/marcotulio956/consensus.algoIC/blob/master/img/rzvs_leader-1d_20agents.png)

### Rendezvous of 20 Integrating Agents with a Moving Leader
![alt text](https://github.com/marcotulio956/consensus.algoIC/blob/master/img/rzvs_sin_leader-1d_20agents.png)

### Rendezvous of 17 Integrating Agents with a Leader 2D
![alt text](https://github.com/marcotulio956/consensus.algoIC/blob/master/img/rzvs_leader-2d_17agents.png)

### Rendezvous of 5 Integrating Agents with a Moving Leader 3D
![alt text](https://github.com/marcotulio956/consensus.algoIC/blob/master/img/rzvs_circular_leader-3d_5agents.png)

### Integrators in SS
**Single Integrator**
  - System 
<img src="https://latex.codecogs.com/svg.image?x'(t)=u(t)\\x(t)=\int&space;u(t)&space;\\x_1=x,x_2=x'&space;\\x'_1=x_2,x'_2=u(t)&space;\\"/> 
       
  - PI Control Law
<img src="https://latex.codecogs.com/svg.image?u_k(t)=E_P&space;&plus;&space;E_I(t),\quad&space;k=[2,n]\\E_P=K_p\sum^n_{i=1}w_{i,0}.e_{i,0}=K_p\sum^n_{i=1}w_{i,0}(x_i-x_0)\\E_I(t)=K_i\sum_{i=-\infty}^t\sum^n_{i=1}&space;w_{i,0}.e_{i,0}(t)=K_i\sum_{i=-\infty}^t\sum^n_{i=1}&space;w_{i,0}.(x_i(t)-x_0(t))\\&space;"/> 
        
**Double Integrator**
  - System
<img src="https://latex.codecogs.com/svg.image?x''(t)=u(t)&space;\\x(t)=\int\int&space;u(t)&space;\\x_1=x,&space;x_2=x'\\x_1'=x_2,x_2'=u(t)&space;"/> 

  - P Control Law
<img src="https://latex.codecogs.com/svg.image?u_k(t)=K_p\sum^n_{i=1}w_{i,0}.e_{i,0}=K_p\sum^n_{i=1}w_{i,0}(x_i-x_0),&space;\quad&space;k=[2,n]"/> 

  - Viscous Term
  

### Single vs Double Integrator in 3D with a Helical Leader
#### 4 Agents
![alt text](https://github.com/marcotulio956/consensus.algoIC/blob/master/img/rzvs_ia_vs_dia_4agents.png)
#### 10 Agents
![alt text](https://github.com/marcotulio956/consensus.algoIC/blob/master/img/rzvs_ia_vs_dia_10agents.png)


### Boids with 200 Agents, Tunned
- Centering with Neighbours
- Colision Avoidance
- Matching Speeds
- Bounded

<img src="https://github.com/marcotulio956/consensus.algoIC/blob/master/img/boids_tunned_200agents.gif" alt="My Project GIF" width="640" height="480">
