def adjust_angle(dist, nums):
    magrin = 0.1
    for num in nums:
        lower = num - magrin
        upper = num + magrin
        print(lower, upper, (lower <= dist), (upper >= dist))
        if (lower <= dist) and (upper >= dist):
            return True
    return False

dist = 10.0234189653
print(adjust_angle(dist, [0.7, 1, 3, 5, 10, 20]))