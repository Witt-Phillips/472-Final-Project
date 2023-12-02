import numpy as np
import matplotlib.pyplot as plt

# Initialize the prior as a 4x4 dually stochastic matrix
prior = np.ones((4, 4)) / 4

initial_posterior = np.array([
    [.75, 0, .25, 0],
    [0, .25, .75, 0],
    [0, .75, 0, .25],
    [.25, 0, 0, .75]
])

nsamples = 100
count = np.zeros((4, 4))
draws = np.full((nsamples, 2), np.nan)

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

posterior = random_posterior()

for i in range(nsamples):
    draw = drawfromposterior(posterior)
    draws[i, :] = draw
    count[draw[0], draw[1]] += 1
    prior = update_prior(draw, prior)

# Display the final updated prior
plt.subplot(131)
plt.imshow(posterior)
plt.title("Posterior")

plt.subplot(132)
plt.imshow(count)
plt.title("Berries Seen")

plt.subplot(133)
plt.imshow(prior)
plt.title("Prior")

plt.tight_layout()
plt.show()
