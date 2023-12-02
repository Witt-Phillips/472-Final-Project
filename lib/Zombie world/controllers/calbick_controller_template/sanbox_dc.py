import numpy as np
import matplotlib.pyplot as plt
import pandas as pd

rows = ["red", "pink", "orange", "yellow"]
cols = ["p20E", "m20E", "p40H", "arm"]

#Classes
class Map:
    def __init__(self):
        self.prior = np.ones((4, 4)) / 4
        self.count = np.zeros((4, 4))
        self.draws = np.empty((0, 2))

# Initialize the prior as a 4x4 dually stochastic matrix
prior = np.ones((4, 4)) / 4

initial_posterior = np.array([
    [.75, 0, .25, 0],
    [0, .25, .75, 0],
    [0, .75, 0, .25],
    [.25, 0, 0, .75]
])

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


#Simulate
posterior = random_posterior()
nsamples = 0
count = np.zeros((4, 4))
#draws = np.full((nsamples, 2), np.nan)
draws = np.empty((0, 2))

for i in range(nsamples):
    draw = drawfromposterior(posterior)
    print(draw)
    #draws[i, :] = draw
    draws = np.vstack((draws, draw))
    print(draws)
    count[draw[0], draw[1]] += 1
    prior = update_prior(draw, prior)

count, draws, prior = observe_berry("red", "p40H", count, draws, prior)
count, draws, prior = observe_berry("red", "p20E", count, draws, prior)
count, draws, prior = observe_berry("pink", "p20E", count, draws, prior)



print(to_df(posterior))
print(to_df(prior))
print(to_df(count))

if 1:
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