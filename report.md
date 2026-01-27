# Part 1:
1. You can determine the length of ech link of the robot arm. The base link is **0.05m**, the shoulder link is **0.15m**, the upper arm link is **0.56m**, the forearm link is **0.5m**, all of the wrist links are **0.12m**. There is an ee_link which does not have a length listed. I think the maximum range of the arm is roughly **1.06m**, with maybe a little more range depending on how the wrists are angled, that is to say, it could reach an item **1.06m** away from it's base.

# Part 2:
2. The arm could not quite reach my estimated position. It seemed like it could reach just shy of a full meter. I assume this is because to move straight away from the base each segment of the arm needs to be angled slightly  so it loses some distance. I also did not properly account for the length of each of the joints.

3.  
