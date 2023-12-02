import random
import math

#Berry Data Structure
class Map:
    def __init__(self):
        #Added type, sisterBerry, and timesSeen
        props   = {"type": None, "probability": 0.5, "sister_color": None, "times_seen": 0}
        effects = {"effect1":props, "effect2":props}
        self.berryProbabilities   = {
                "red": effects,
                "pink": effects,
                "orange": effects,
                "yellow": effects
            }

class BerryObject:
    def __init__(self, id, berry_color=None, effect=None):
        self.id = id
        self.effect = effect
        self.color     = berry_color
        #self.first_seen  = None
        #self.second_seen = None

# Bayesian Update --------------------------------------------------------------------------------

#Prior - chance that first berrry we encountered was primary. Changes as we make observations.
prior = 0.5

# Updates the prior based on evidence
def bayesian_update(prior, likelihood):
    ltp = likelihood * prior
    marginal = ltp + ((1 - likelihood) * (1 - prior))
    return ltp / marginal
#-------------------------------------------------------------------------------

#Likelihood (Binomial Theorem)
def likelihood(k, n, p):
    return math.comb(n, k) * ((p) ** k) * ((p - 1) ** (n - k))

#Basic Update Simulation -------------------------------------------------------------------------------
sample_size = 10
weights = [0.75, 0.25]
observations = [random.choices([1, 0], weights=weights)[0] for _ in range(sample_size)]
#print(observations)

for ob in observations:
        p = 0.75 if ob == 1 else 0.25
        l = likelihood(1, 1, .75)
        prior = bayesian_update(prior, l)
        #print("ob", ob, "\nprior:", prior)

#print("\nFinal prior:", prior)
#print(observations.count(1))

#Update Map
def update_berry_prob(map, berry_obj):
     color = berry_obj.color
     observed_effect = berry_obj.effect
     
     print(color)
     #print(observed_effect)

     for effect_num, effect in map.berryProbabilities[color].items():
        #print(effect_num)
        if effect["type"] == None:
            effect["type"] = observed_effect
            print(effect["type"])
        if effect["type"] == observed_effect:
            #Probability update
            effect["probability"] = bayesian_update(effect["probability"], likelihood(1, 1, p))
            other_effect = "effect2" if effect_num == "effect1" else "effect1"
            map.berryProbabilities[color][other_effect]["probability"] = bayesian_update(effect["probability"], likelihood(0, 1, p))
            effect["times_seen"] += 1
            break
                

# Integrated Simulation with linked probabilities -------------------------------------------------------------------------------
p = 0.75
map = Map()
berryList = []
berry_color_options = ["red", "pink", "orange", "yellow"]
effect_options = ["p20E", "m20E", "p40H", "arm"]

while sample_size > 0:
     color = random.choice(berry_color_options)
     effect = random.choice(effect_options)
     id = ''.join(random.choices('0123456789', k=4))
     berryList.append(BerryObject(id, color, effect))
     sample_size -= 1

for item in berryList:
     #print(item.effect)
     update_berry_prob(map, item)

print(map.berryProbabilities)


