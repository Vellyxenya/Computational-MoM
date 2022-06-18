## Infos

First Name: Noureddine

Last Name: Gueddach

---

## Simulations

### 1) **FEM**

<img style="display: block; margin-left: auto; margin-right: auto; width: 80%;"
      src="res/fem.gif"
      alt="Evaluating integration schemes">
</img>

### 2) **Soft manipulation**

<img style="display: block; margin-left: auto; margin-right: auto; width: 80%;"
      src="res/manip.gif"
      alt="Evaluating integration schemes">
</img>

### 2) **Soft manipulation with regularizers**

More details about the regularizers are show under *question 11*

  * **Distance difference regularizer:** Works but might be a bit slow

<img style="display: block; margin-left: auto; margin-right: auto; width: 80%;"
      src="res/regularizer_dist_diff_x10.gif"
      alt="Evaluating integration schemes">
</img>

  * **Maximum minus Mean energy regularizer:** This seems to work the best.

<img style="display: block; margin-left: auto; margin-right: auto; width: 80%;"
      src="res/regularizer_max_mean_x10.gif"
      alt="Evaluating integration schemes">
</img>

**Note I:** the videos are sped up x10

**Note II:** I noticed that putting the control points further apart helps the optimizer as it tends to find a good solution quicker.

---

## Theory Questions

**Solution to Question 4:**

![Solution to question 4](res/cmm_a4_q4.png)

**Solution to Question 10:**

The objective only includes the position difference between the current feature point positions and their target positions. It is easy to see that there are infinitly many handle configurations that minimize the objective function. In other words, the objective/energy landscape has a non-trivial null-space on which the gradient is flat, so the optimizer has no incentive to go and explore other configurations, since as we know gradient descent stops updating when it receives flat gradients.

This being the case, the solution found by the optimizer is heavily depending on the initial configuration (vertically aligned pins in our case). Since it can minimize the objective without changing the pin orientations, it does not bother changing them.

**Solution to Question 11:**

We need a way to include a penalty term in the objective that penalizes the deformations at the extremities. There are many possible options. I list below the ones that I implemented:

![Solution to question 11](res/cmm_a4_q11.png)

