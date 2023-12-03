# Agent-based-modelling
Repository containing and briefly explaining the code to my agent-based modelling project for my Aerospace Engineering master. The goal of the project is to have agents that can reach their goal position from their starting position in an optimal way. Wghile doing this the agents have to communicate with their surroundings and other agents to ensure an optimal and feasible solution. I did this in collaboration with my project mate Daniël Timmermans, most of the code we wrote together, however parts of the framework were already supplied during the assignment (many parts of the supplied framework has been heavily edited though). 

For the creation of this program 3 different solver models have been created. All three based on different inherent priority ruling and planning systems. The three models consist of, firstly the Prioritized Planning model (see prioritized.py), secondly the conflict-based search model (see cbs.py) and lastly the completely self designed distributed model (see distributed). All three models can be used for the solving of the problems on different maps (see the instances or the Dtest_randomizer that randomly generates maps).

For the testing of these models a Monte Carlo simulation has been set up (see monte_carlo.ipynb) together with a random map and agent generator (see D_test_randomizer.py). This way the models could be vividly tested for robustness and performance. For further analysis and results see the PDF with the report about the models.

If you would like to (quickly) test the models, run the following command into your terminal:
python run_experiments.py --instance instances/Dtest_function_updated.txt --solver Distributed
Here a map created by the Dtest_function will be tested together with the Distributed solver (Distributed could be replaced with CBS or Prioritized to test the different solvers). If a different map would want to tested, insert a different instance name from the instances folder into this command. 

For any further questions about the code, feel free to contact me at duncanvanwoerkom@gmail.com 

