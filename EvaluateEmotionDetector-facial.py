import cv2
import numpy as np
from tensorflow.keras.models import model_from_json
import matplotlib.pyplot as plt
from tensorflow.keras.preprocessing.image import ImageDataGenerator
from sklearn.metrics import confusion_matrix, classification_report, ConfusionMatrixDisplay

# Define emotion dictionary
emotion_dict = {0: "Angry", 1: "Disgusted", 2: "Fearful", 3: "Happy", 4: "Neutral", 5: "Sad", 6: "Surprised"}

# Load model structure and weights
json_file = open('emotion_model.json', 'r')
loaded_model_json = json_file.read()
json_file.close()
emotion_model = model_from_json(loaded_model_json)
emotion_model.load_weights("emotion_model.weights.h5")
print("Loaded model from disk")

# Initialize image data generator with rescaling
test_data_gen = ImageDataGenerator(rescale=1./255)

# Preprocess all test images
test_generator = test_data_gen.flow_from_directory(
    'data/test',
    target_size=(48, 48),
    batch_size=64,
    color_mode="grayscale",
    class_mode='categorical',
    shuffle=False  # Important to keep the order for evaluation
)

# Do prediction on test data
predictions = emotion_model.predict(test_generator, steps=test_generator.samples // test_generator.batch_size + 1)

# Get true labels
true_labels = test_generator.classes

# Get predicted labels
predicted_labels = np.argmax(predictions, axis=1)

# Print confusion matrix
print("Confusion Matrix for Emotion Recognition")
c_matrix = confusion_matrix(true_labels, predicted_labels)
print(c_matrix)
cm_display = ConfusionMatrixDisplay(confusion_matrix=c_matrix, display_labels=[emotion_dict[i] for i in range(7)])
cm_display.plot(cmap=plt.cm.Blues)
plt.show()

# Print classification report
print("Classification Report for Emotion Recognition")
print(classification_report(true_labels, predicted_labels, target_names=[emotion_dict[i] for i in range(7)]))

