
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

# plt.ion()
plt.show()
plt.tight_layout()