# Read the concatenated lines from the file into memory
with open("Data_Set_SF_Predictor.csv", "r") as file:
    concatenated_lines = file.readlines()

# Open the same file for writing, truncating it to empty it
with open("Data_Set_SF_Predictor.csv", "w") as file:
    # Iterate over the concatenated lines, split them, and write back the original lines to the file
    for line in concatenated_lines:
        # Split the concatenated line into two separate lines
        original_lines = line.strip().split(',', 1)
        
        # Write the original lines back to the file
        for original_line in original_lines:
            file.write(f"{original_line}\n")

