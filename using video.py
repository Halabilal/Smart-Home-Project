import librosa
import numpy as np
import pandas as pd
import keras
from keras.models import load_model, model_from_json
from sklearn.preprocessing import StandardScaler, OneHotEncoder
import cv2
from collections import Counter
import moviepy.editor as mp
import serial
import time

# Load the trained speech model
speech_model = load_model('C:/Users/halaj/OneDrive/Desktop/speech/speech_model.keras')

# Load the scaler and encoder
scaler = StandardScaler()
encoder = OneHotEncoder()

# Fit the scaler and encoder with the same data used during training
features = pd.read_csv('C:/Users/halaj/OneDrive/Desktop/data/features.csv')
X = features.iloc[:, :-1].values
Y = features['labels'].values
scaler.fit(X)
encoder.fit(np.array(Y).reshape(-1, 1))

def extract_features(data, sample_rate):
    result = np.array([])

    # ZCR
    zcr = np.mean(librosa.feature.zero_crossing_rate(y=data).T, axis=0)
    result = np.hstack((result, zcr))

    # Chroma_stft
    stft = np.abs(librosa.stft(data))
    chroma_stft = np.mean(librosa.feature.chroma_stft(S=stft, sr=sample_rate).T, axis=0)
    result = np.hstack((result, chroma_stft))

    # MFCC
    mfcc = np.mean(librosa.feature.mfcc(y=data, sr=sample_rate).T, axis=0)
    result = np.hstack((result, mfcc))

    # RMS
    rms = np.mean(librosa.feature.rms(y=data).T, axis=0)
    result = np.hstack((result, rms))

    # MelSpectrogram
    mel = np.mean(librosa.feature.melspectrogram(y=data, sr=sample_rate).T, axis=0)
    result = np.hstack((result, mel))

    return result

def extract_audio_from_video(video_path):
    video = mp.VideoFileClip(video_path)
    audio = video.audio
    audio_path = "temp_audio.wav"
    audio.write_audiofile(audio_path)
    return audio_path

def load_audio(audio_path, sample_rate=22050):
    data, sr = librosa.load(audio_path, sr=sample_rate)
    return data, sr

def predict_speech_emotion(audio, sample_rate):
    features = extract_features(audio, sample_rate)
    features = scaler.transform([features])
    features = np.expand_dims(features, axis=2)
    prediction = speech_model.predict(features)
    predicted_emotion = encoder.inverse_transform(prediction)
    return predicted_emotion[0][0]

json_file = open(r'C:\Users\halaj\OneDrive\Desktop\emotion detection\emotion_model.json', 'r')
loaded_model_json = json_file.read()
json_file.close()
emotion_model = model_from_json(loaded_model_json)
emotion_model.load_weights(r"C:\Users\halaj\OneDrive\Desktop\emotion detection\emotion_model.weights.h5")
print("Loaded model from disk")

# Facial emotion dictionary
emotion_dict = {0: "angry", 1: "disgusted", 2: "fear", 3: "happy", 4: "neutral", 5: "sad", 6: "surprised"}

def predict_facial_emotion(frame, face_detector):
    gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    num_faces = face_detector.detectMultiScale(gray_frame, scaleFactor=1.3, minNeighbors=5)
    emotions = []
    for (x, y, w, h) in num_faces:
        roi_gray_frame = gray_frame[y:y + h, x:x + w]
        cropped_img = np.expand_dims(np.expand_dims(cv2.resize(roi_gray_frame, (48, 48)), -1), 0)
        emotion_prediction = emotion_model.predict(cropped_img)
        maxindex = int(np.argmax(emotion_prediction))
        emotions.append(emotion_dict[maxindex])
    return emotions

def send_emotion_to_arduino(emotion):
    port = 'COM4'
    baud_rate = 9600
    ser = serial.Serial(port, baud_rate)
    time.sleep(2)  # Wait for the connection to establish

    emotion_code = {
        "angry": "0",
        "disgusted": "1",
        "fear": "2",
        "happy": "3",
        "neutral": "4",
        "sad": "5",
        "surprised": "6",
        "Mismatch": "7"
    }

    if emotion in emotion_code:
        print(f"Sending emotion: {emotion_code[emotion]}")
        ser.write(emotion_code[emotion].encode())
    else:
        print("Sending emotion: 7")
        ser.write(emotion_code["Mismatch"].encode())
    ser.close()

    
def main(video_path):
    # Extract audio from video and predict speech emotion
    audio_path = extract_audio_from_video(video_path)
    audio, sample_rate = load_audio(audio_path)
    speech_emotion = predict_speech_emotion(audio, sample_rate)
    print(f"Predicted Speech Emotion: {speech_emotion}")

    # Process video frames and predict facial emotion
    cap = cv2.VideoCapture(video_path)
    face_detector = cv2.CascadeClassifier(r"C:\Users\halaj\OneDrive\Desktop\emotion detection\haarcascades\haarcascade_frontalface_default.xml")
    facial_emotions = []

    frames_to_capture = 70
    frames_captured = 0

    while frames_captured < frames_to_capture:
        ret, frame = cap.read()
        if not ret:
            break
        emotions = predict_facial_emotion(frame, face_detector)
        if emotions:
            facial_emotions.extend(emotions)
            frames_captured += 1
        cv2.imshow('Emotion Detection', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

    # Determine the most frequent facial emotion
    if facial_emotions:
        most_common_facial_emotion = Counter(facial_emotions).most_common(1)[0][0]
        print(f"Predicted Facial Emotion: {most_common_facial_emotion}")
    else:
        most_common_facial_emotion = None

    # Combine the results
    if most_common_facial_emotion and most_common_facial_emotion == speech_emotion:
        combined_emotion = most_common_facial_emotion
        print(f"Combined Emotion: {combined_emotion}")
    else:
        combined_emotion = "Mismatch"
        print("Emotion detection failed: The models do not agree.")

    # Send the combined emotion to Arduino
    send_emotion_to_arduino(combined_emotion)

if __name__ == "__main__":
    #video_path = r"C:\Users\halaj\OneDrive\Desktop\datare\Actor_01\01-01-06-02-01-02-01.mp4" #fear
    video_path = r"C:\Users\halaj\OneDrive\Desktop\datare\Actor_02\01-01-04-02-02-01-02.mp4"
    main(video_path)
