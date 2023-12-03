import random

def generate_agents_on_map(input_file_path, output_file_path, num_agents, seed_number=42, working_check=False):
    random.seed(seed_number)
    
    try:
        with open(input_file_path, 'r') as input_file:
            lines = input_file.readlines()

            # Grid size
            num_rows, num_columns = map(int, lines[0].strip().split())

            # Map reading
            map_content = [[cell for cell in line.strip() if cell in ".@"]
               for line in lines[1:1 + num_rows]]
        
            blocked_positions = set((x, y) for y, row in enumerate(map_content) for x, cell in enumerate(row) if cell == "@")
            available_positions = [(x, y) for x in range(num_columns) for y in range(num_rows) if (x, y) not in blocked_positions]

            # Checking the feasability of the number of agents
            if len(available_positions) < num_agents:
                print("Not enough available positions for agents. Exiting.")
                return

            # Randomize the available positions
            random.shuffle(available_positions)                

            # Writing the output file
            with open(output_file_path, 'w') as output_file:
                output_file.write(f"{num_rows} {num_columns}\n")

                # Writing the map
                for line in map_content:
                    output_file.write(" ".join(line) + "\n")

                output_file.write(f"{num_agents}\n")

                # Writing the agents and their start and goal positions
                for agent_id in range(1, num_agents + 1):
                    start_x, start_y = available_positions.pop()
                    goal_x, goal_y = available_positions.pop()
                    output_file.write(f"{start_y} {start_x} {goal_y} {goal_x}\n")

                output_file.write(f"\n \n \n seed = {seed_number}")

    except Exception as e:
        print(f"An error occurred: {str(e)}")

input_file_path = 'instances\\Dtest_situatie_3.txt'  
output_file_path = 'instances\\Dtest_function_updated.txt' 
num_agents = 16
seed_number = 42
generate_agents_on_map(input_file_path, output_file_path, num_agents, seed_number) # random test