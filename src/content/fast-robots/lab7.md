---
title: Lab 7
description: Kalman Filter
pubDate: 2026-03-25
---
<article class="article">

## Estimate drag and momentum

To estimate the drag and momentum terms for my A and B matrices, I used a step response by driving the car towards a wall at a constant imput motor speed while logging motor input values and ToF sensor output. The step responce, u(t), I chose was 150 PWM, as found in my Lab 5 (max speed video).

Using the distance found by the ToF, I found the exponential fit to find the steady state.



## Initialize Kalman Filter (KF)
We can then compute the appropriate KF matrices D and M:


Process and measurement noise needs to be accounted for. 



## Implement KF in Python
```python
def kf(mu, sigma, data_ready, u, y):

    mu_p = A_d.dot(mu) + B_d.dot(u)
    sigma_p = A_d.dot(sigma.dot(A_d.transpose())) + Sigma_u

    if data_ready:
        sigma_m = C.dot(sigma_p.dot(C.transpose())) + Sigma_z
        kkf_gain = sigma_p.dot(C.transpose().dot(np.linalg.inv(sigma_m)))
        y_m = y - C.dot(mu_p)
        mu = mu_p + kkf_gain.dot(y_m)
        sigma = (np.eye(2) - kkf_gain.dot(C)).dot(sigma_p)
    else:
        mu = mu_p
        sigma = sigma_p

    return mu, sigma
```

## Implement KF on the Car


## Acknowledgements

I referenced the Aidan McNay's past lab report from from Spring 2025. 

