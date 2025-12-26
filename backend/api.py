# This is a simplified API file for Vercel deployment
# Import only what's needed to minimize package size

from fastapi import FastAPI
import os

# Create a simple app for testing
app = FastAPI()

@app.get("/")
def read_root():
    return {"message": "FastAPI backend is running on Vercel!"}

@app.get("/health")
def health_check():
    return {"status": "healthy"}