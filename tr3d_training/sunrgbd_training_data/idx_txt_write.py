with open("numbers.txt", "w") as f:
    for i in range(1, 961):
        if (i - 1) % 40 >= 25:
            f.write(str(i) + "\n")

