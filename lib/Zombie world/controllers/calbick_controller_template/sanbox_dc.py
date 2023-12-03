import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

rows = ["red", "pink", "orange", "yellow"]
cols = ["p20E", "m20E", "p40H", "arm"]
weight_names = ["health", "energy", "dist"]
weights = np.array(([[0.5, 0.5, 0.5],
                    [0.5, 0.5, 0.5],
                    [0.5, 0.5, 0.5],
                    [0.5, 0.5, 0.5]]))
#Classes
class Map:
    def __init__(self):
        self.prior = to_df(np.ones((4, 4)) / 4)
        self.count = to_df(np.zeros((4, 4)))
        self.draws = np.empty((0, 2))
        self.weights = pd.DataFrame(weights, index=cols, columns=weight_names)
    
    def observe_berry_map(self, color, effect):
        prior = self.prior.values
        count = self.count.values

        draw = np.array([rows.index(color), cols.index(effect)])
        self.draws = np.vstack((self.draws, draw))
        count[draw[0], draw[1]] += 1
        prior = update_prior(draw, prior)

        self.prior = to_df(prior)
        self.count = to_df(count)

    def priority_score(self, color, health, energy, distance):
        #Get effect1 (likely primary) and effect 2 (likely secondary), and their probabilities
        seen_array = self.count.loc[color, :] != 0
        
        effect_1 = self.prior.iloc[rows.index(color)].idxmax()
        prob_effect_1 = self.prior.at[color, effect_1]
        seen_array[effect_1] = False
        
        
        effect_2 = seen_array.idxmax()
        if seen_array.max() == 0:
            effect_2 = second_largest_column(self.prior.iloc[rows.index(color)])
            #print("effect 2 defaulted:", effect_2)
        prob_effect_2 = self.prior.at[color, effect_2]

        #priority score by effect
        score_1 = priority_score_formula(self.weights.at[effect_1, "health"],
                                         self.weights.at[effect_1, "energy"],
                                         self.weights.at[effect_1, "dist"],
                                         health, energy, distance)
        score_2 = priority_score_formula(self.weights.at[effect_2, "health"],
                                         self.weights.at[effect_2, "energy"],
                                         self.weights.at[effect_2, "dist"],
                                         health, energy, distance)
        
        #replace pro_effect_2 with 1 - prob_effect_1 (should really factor in all possible effects,
        #but will be small and don't really have time...)
        weighted_average = (prob_effect_1 * score_1) + (prob_effect_2 * score_2)
        return weighted_average


def second_largest_column(row):
        return sorted(row.index, key=row.get, reverse=True)[1]
 
def priority_score_formula(w_h, w_e, w_d, h, e, d):
    health_score = score(w_h, h)
    energy_score = score(w_e, e)
    dist_score   = score(w_d, (10 * d))
    return (health_score + energy_score + dist_score) / 30000


def score(weight, value):
    return weight * ((100 - value) ** 2)

def random_posterior():
    shuffled_matrix = initial_posterior.copy()
    np.random.shuffle(shuffled_matrix)  # Shuffle rows
    np.random.shuffle(shuffled_matrix.T)  # Shuffle columns
    return shuffled_matrix

def drawfromposterior(posterior):
    color = np.random.randint(4)
    effects = np.random.rand()
    c = posterior[color, :]
    e = (c == .75) if effects > .25 else (c == .25)
    return np.array([color, np.where(e)[0][0]])

def calculate_likelihoods(observed_color, observed_effect, prior):
    likelihoods = np.zeros_like(prior)
    for color in range(prior.shape[0]):
        for effect in range(prior.shape[1]):
            if color == observed_color:
                likelihoods[color, effect] = 0.75 if effect == observed_effect else 0.25
            else:
                likelihoods[color, effect] = 0.1 if effect == observed_effect else 0.5
    likelihoods /= np.sum(likelihoods)
    return likelihoods

def update_prior(obs, prior):
    likelihoods = calculate_likelihoods(obs[0], obs[1], prior)
    updated_prior = prior * likelihoods
    updated_prior /= np.sum(updated_prior, axis=1, keepdims=True)  # Normalize rows
    updated_prior /= np.sum(updated_prior, axis=0, keepdims=True)  # Normalize columns
    return updated_prior

def observe_berry(color, effect, count, draws, prior):
    draw = np.array([rows.index(color), cols.index(effect)])
    draws = np.vstack((draws, draw))
    count[draw[0], draw[1]] += 1
    prior = update_prior(draw, prior)
    return count, draws, prior

def to_df(matrix):
    return pd.DataFrame(matrix, index=rows, columns=cols)


#Simulate using data structure
initial_posterior = np.array([
    [.75, 0, .25, 0],
    [0, .25, .75, 0],
    [0, .75, 0, .25],
    [.25, 0, 0, .75]
])

posterior = random_posterior()

map = Map()
nsamples = 2
# print(map.prior)
# print(map.weights)

for i in range(nsamples):
    draw = drawfromposterior(posterior)
    color = rows[draw[0]]
    effect = cols[draw[1]]
    map.observe_berry_map(color, effect)

print(map.count)
print(map.prior)

print("Priority score: Red, H:50 E:50 D: 1", map.priority_score("red", 100, 100, 1))
print("Priority score: Red, H:50 E:50 D: 1", map.priority_score("red", 50, 100, 1))
print("Priority score: Red, H:50 E:50 D: 1", map.priority_score("red", 100, 50, 1))
print("Priority score: Red, H:50 E:50 D: 1", map.priority_score("red", 10, 100, .1))
print("Priority score: Red, H:50 E:50 D: 1", map.priority_score("red", 100, 10, .1))
print("Priority score: Orange, H:50 E:50 D: 1", map.priority_score("orange", 100, 10, .1))


#Plotting
count = map.count
prior = map.prior

if 0:
    # Create subplots
    fig, axes = plt.subplots(1, 3, figsize=(25, 8))

    # Display the final updated prior
    axes[0].imshow(posterior)
    axes[0].set_title("Posterior")

    axes[1].imshow(count)
    axes[1].set_title("Berries Seen")

    axes[2].imshow(prior)
    axes[2].set_title("Prior")

    # Set buffer space above the main title
    title_buffer_space = 0.1  # Adjust this value as needed
    fig.subplots_adjust(top=1 - title_buffer_space)

    # Set the main title
    title = "Berry Probability Model\n n =" + str(nsamples)
    plt.suptitle(title, fontsize=16, y = .9)

    # Display the plot
    plt.tight_layout()
    plt.show()