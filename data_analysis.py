import pandas as pd
import seaborn as sns
import matplotlib.pyplot as plt
import re
from scipy.stats import ttest_ind

def parse_line(line):
    """Parse a single line of the data file with the adjusted regular expression."""
    if "Missing data" in line:
        return None
    match = re.search(r'Cost: (\d+), Makespan: +(\d+) \(agent (\d+)\), CPU time: (\d+\.\d+), Goal priority: (\w+), iteration: (\d+)', line)     
    
    if match:
        return {
            'Cost': int(match.group(1)),                    
            'Makespan': int(match.group(2)),
            'Agent': int(match.group(3)),
            'CPU_Time': float(match.group(4)),
            'Goal_Priority': match.group(5),
            'Iteration': int(match.group(6))
        }
    return None

def read_data(file_path):
    """Read and parse the data from the file."""
    data = []
    with open(file_path, 'r') as file:
        for line in file:
            parsed_data = parse_line(line)              # Parse each line of the file
            if parsed_data:
                data.append(parsed_data)                # Append the parsed data to the list
    return pd.DataFrame(data)

def plot_data(df, solver_type):
    """Create box plots for Cost, Makespan, and CPU Time."""
    sns.set(style="whitegrid")
    plt.figure(figsize=(18, 6))

    plt.subplot(1, 3, 1)
    sns.boxplot(y=df['Cost'])
    plt.title(f'{solver_type} Box Plot of Cost')

    plt.subplot(1, 3, 2)
    sns.boxplot(y=df['Makespan'])
    plt.title(f'{solver_type} Box Plot of Makespan')

    plt.subplot(1, 3, 3)
    sns.boxplot(y=df['CPU_Time'])
    plt.title(f'{solver_type} Box Plot of CPU Time')

    plt.tight_layout()
    plt.show()

def analyze_data(df, solver_type):
    """Perform basic statistical analysis, excluding 'Iteration', and explore correlations."""
    # Excluding 'Iteration' from the analysis
    df_analysis = df.drop(columns=['Iteration', 'Agent'])

    # Basic Statistics
    print("Basic Statistics:\n", df_analysis.describe())

    # Correlation Matrix
    correlation_matrix = df_analysis.corr()
    print("\nCorrelation Matrix:\n", correlation_matrix)

    # Pair Plot
    sns.pairplot(df_analysis, kind="scatter")
    plt.show()


if __name__ == "__main__":
    solver_type = "Prioritized"
    file_path = f"MC_results\MC_results_{solver_type}_iterations_100.txt" 
    df = read_data(file_path)
    plot_data(df, solver_type)
    analyze_data(df, solver_type)

