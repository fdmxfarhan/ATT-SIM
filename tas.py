import random

min_val=1
max_val=6

roll_again= "yes"

while roll_again == "yes" or roll_again== "y":
    print("rolling the dices...")
    print("the values are:")


    print(random.randint(min_val,max_val))
    terminate the loop
    roll_again = input("roll the dices again?")

     