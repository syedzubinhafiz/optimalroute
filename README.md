# Optimal Route

Caffeine runs through your veins and you can’t function without it. No matter where you go, you would need to grab a cup of coffee on the way to the destination. You, however, often find yourself late to your destination due to this; especially when the cafes have a long waiting time. 
With that, you have downloaded the travel time between key locations in your city which you can use to estimate the travel time from a point to another. You have also time the waiting time of each cafe that you would grab your coffee from.
• Therefore we create a RoadGraph class :
- The init method of RoadGraph would take as an input a list of roads roads represented as a list of tuples (u,v,w) where: 
    • u is the starting location ID for a road, represented as a non-negative integer. 
    
    • v is the ending location ID for a road, represented as a non-negative integer.
    
    • w is the time taken to travel from location u to location v, represented as a non-negative integer. 
    
    • We didn't assume that the list of tuples are in any specific order.
    
    • We didn't assume that the roads are 2-way roads.

 We can then calculate the optimal routes for your commute while grabbing coffee along the way using a function in the RoadGraph class called routing(self, start, end). The init method of RoadGraph also takes as an input a list of cafes cafes represented as a list of tuples (location,waiting_time) where:

• Location is the location of the cafe; represented as a non-negative integer. 

• waiting_time is the waiting time for a coffee in the cafe, represented as a non-negative integer. 

• We didn't assume that the list of tuples are in any specific order. 

• We assumed that all of the location values are from the set {0, 1, ..., |V|-1}. 

• We assumed that all of the location values are unique. 

• We didn't assume waiting_time to be within any range except that it is a value > 0.
