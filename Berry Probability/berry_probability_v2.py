import random
import math
import numpy as np
import pandas as pd

rows = ["red", "pink", "orange", "yellow"]
cols = ["p20E", "m20E", "p40H", "arm"]

#Data structure
class Map:
    def __init__(self):
        #Map Structure: rows: red, pink, orange, yellow; columns: "p20E", "m20E", "p40H", "arm"
        self.berry_prob_matrix = to_df(np.full((4,4), 0.25))
        self.berries_seen_arr = to_df(np.full((4,4), False))

    def observe_berry(self, color, effect):
        # Set/ normalize 0 for ruled out cases if first time seeing effect for color
        if self.berries_seen_arr.at[color, effect] == False:
            #print("Saw", color, effect, "for first time")
            self.berries_seen_arr.at[color, effect] = True
            
            #identify rows and cols that can be deduced
            print(self.berries_seen_arr)

            bool_row_sums = pd.DataFrame(self.berries_seen_arr.sum(axis = 1), columns=['Sum'])
            bool_col_sums = pd.DataFrame(self.berries_seen_arr.sum(axis = 0), columns=['Sum'])
            print(bool_row_sums)
            print(bool_col_sums)

            #Case where we are adding to deduceable row:
            known_rows = [idx for idx, value in enumerate(bool_row_sums) if value == 2]
            known_cols = [idx for idx, value in enumerate(bool_col_sums) if value == 2]
            
            mask_matrix = ~self.berries_seen_arr.values
            to_mask = self.berry_prob_matrix.values

            for row in known_rows:
                to_mask[row, mask_matrix[row, :]] = 0.0
            for col in known_cols:
                to_mask[mask_matrix[:, col], col] = 0.0

            self.berry_prob_matrix = normalize(self.berry_prob_matrix)
            # print(mask_matrix)
            print("Known rows:", known_rows)
            # print("Known cols:", known_cols)

            #if the current row isn't known
            if index_number = df.index.get_loc(target_row_label)
            #if the current row is now known 
            self.berry_prob_matrix.at[color, effect] = bayesian_update(self.berry_prob_matrix.at[color, effect], 0.75)
            print("Pre-normalization", self.berry_prob_matrix.at[color, effect])
            return

        #Bayesian update and normalize given current observed berry
        self.berry_prob_matrix.at[color, effect] = bayesian_update(self.berry_prob_matrix.at[color, effect], 0.75)
        print("Pre-normalization", self.berry_prob_matrix.at[color, effect])
        self.berry_prob_matrix = normalize(self.berry_prob_matrix)

def to_df(matrix):
    return pd.DataFrame(matrix, index=rows, columns=cols)

#Normalize doubly-stochastic matrix
def normalize(matrix):
    np_matrix = matrix.values

    arr_row_sums = np_matrix.sum(axis = 1)[:, np.newaxis]
    norm_matrix = np_matrix / arr_row_sums

    arr_col_sums = norm_matrix.sum(axis = 0)
    norm_matrix /= arr_col_sums

    return to_df(norm_matrix)



# Bayesian Update. Takes prior, likelihood, returns posterior.
def bayesian_update(prior, likelihood):
    ltp = likelihood * prior
    marginal = ltp + ((1 - likelihood) * (1 - prior))
    return ltp / marginal

#Likelihood (Binomial Theorem) - for our cases, should always be 0.75
def likelihood(k, n, p):
    return math.comb(n, k) * ((p) ** k) * ((p - 1) ** (n - k))

#Simulation
map = Map()
#map.berry_prob_matrix.at['red', 'p20E'] = .75
#print(map.berry_prob_matrix)
map.observe_berry('red', 'p20E')
# map.observe_berry('red', 'm20E')
# map.observe_berry('pink', 'm20E')
# map.observe_berry('orange', 'p40H')
# map.observe_berry('orange', 'p40H')
# map.observe_berry('yellow', 'arm')
# map.observe_berry('pink', 'arm')
#map.observe_berry('yellow', 'p40H')




# map.observe_berry('pink', 'p20E')
print("Berries Seen Matrix:\n", map.berries_seen_arr)
print("Probability Matrix:\n", map.berry_prob_matrix)
#map.berry_prob_matrix.at['red', 'p20E'] = None

print("Bayesian update 0.5, .75", bayesian_update(0.25, 0.75))