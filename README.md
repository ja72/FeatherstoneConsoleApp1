# FeatherstoneConsoleApp1

A C# implementation of the Featherstone recursvie algorithm for solving the dynamics of serial chains of rigid bodies and/or open tree structures of rigid bodies.

## Basic Equations in Spatial Notation

### Joint Axis

$$\boldsymbol{s}_{i}=\begin{bmatrix}\vec{r}_{i}\times\hat{z}_{i}\\
\hat{z}_{i} \end{bmatrix}\,{\rm or}\,\begin{bmatrix}\hat{z}_{i}\\
\\ \end{bmatrix}$$

### Spatial Inertia

$${\bf I}_{i}=\begin{bmatrix}m_{i} & -m_{i}[\vec{r}_{i}^{C}\times]\\
m_{i}[\vec{r}_{i}^{C}\times] & \mathcal{I}_{i}-m_{i}[\vec{r}_{i}^{C}\times][\vec{r}_{i}^{C}\times]
\end{bmatrix}$$

### Rigid Body Mechanics
$$\boldsymbol{f}_{i}-\boldsymbol{f}_{i+1}+\boldsymbol{w}_{i}={\bf I}_{i}\boldsymbol{a}_{i}+\boldsymbol{v}_{i}\times{\bf I}_{i}\boldsymbol{v}_{i}$$

### Joint Conditions
$$Q_{i}=\boldsymbol{s}_{i}^{\top}\boldsymbol{f}_{i}$$

## Kinematics

Loop up the chain `i=1 to N`

Velocity Kinematics
$$\boldsymbol{v}_{i}=\boldsymbol{v}_{i-1}+\boldsymbol{s}_{i}\dot{q}_{i}$$

Start with 
$$\boldsymbol{v}_{0}=\begin{bmatrix}0\\ 0 \end{bmatrix}$$

Bias Acceleration
$$\boldsymbol{\kappa}_{i}=\begin{bmatrix}\vec{\omega}_{i}\times & \vec{v}_{i}\times\\ 0
 & \vec{\omega}_{i}\times
\end{bmatrix}\boldsymbol{s}_{i}\dot{q}_{i}$$

Bias Foces
$$\boldsymbol{p}_{i}=\begin{bmatrix}\vec{\omega}_{i}\times & 0\\
\vec{v}_{i}\times & \vec{\omega}_{i}\times
\end{bmatrix}{\bf I}_{i}\boldsymbol{v}_{i}$$

Weight
$$\boldsymbol{w}_{i}=\begin{bmatrix}m_{i}\vec{g}\\
\vec{r}_{i}^{C}\times m_{i}\vec{g}
\end{bmatrix}$$

## Articulated Inertia

Loop down the chain `i=N to 1`

Articulated Inertia
$${\bf I}_{i}^{A}={\bf I}_{i}+\sum_{n}^{{\rm children}}\left(1-\boldsymbol{T}_{n}\boldsymbol{s}_{n}^{\top}\right){\bf I}_{n}^{A}$$

Articulated Bias Force
$$\boldsymbol{p}_{i}^{A}=\boldsymbol{p}_{i}-\boldsymbol{w}_{i}+\sum_{n}^{{\rm children}}\left(\boldsymbol{T}_{n}Q_{n}+\left(1-\boldsymbol{T}_{n}\boldsymbol{s}_{n}^{\top}\right)\left({\bf I}_{n}^{A}\boldsymbol{\kappa}_{n}+\boldsymbol{p}_{n}^{A}\right)\right)$$

Start with
$$\begin{aligned}{\bf I}_{N}^{A} & ={\bf I}_{N}\\
\boldsymbol{p}_{N}^{A} & =\boldsymbol{p}_{N}-\boldsymbol{w}_{N}
\end{aligned}$$

## Dynamics Solution

Loop up the chain `i=1 to N`

Joint Acceleration Solution
$$\ddot{q}_{i}=\left(\boldsymbol{s}_{i}^{\top}{\bf I}_{n}^{A}\boldsymbol{s}_{i}\right)^{-1}\left(Q_{i}-\boldsymbol{s}_{i}^{\top}\left({\bf I}_{n}^{A}\left(\boldsymbol{a}_{i-1}+\boldsymbol{\kappa}_{i}\right)+\boldsymbol{p}_{i}^{A}\right)\right)$$

Acceleration Kinematics
$$\boldsymbol{a}_{i}=\boldsymbol{a}_{i-1}+\boldsymbol{s}_{i}\ddot{q}_{i}+\boldsymbol{v}_{i}\times\boldsymbol{s}_{i}\dot{q}_{i}$$

Joint Forces
$$\boldsymbol{f}_{i}={\bf I}_{n}^{A}\boldsymbol{a}_{i}+\boldsymbol{p}_{i}^{A}$$

