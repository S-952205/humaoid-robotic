import os
import sys

# Set the TESTING environment variable to true
os.environ['TESTING'] = 'true'

# Now run the uvicorn server
if __name__ == "__main__":
    import uvicorn
    from main import app

    print("Starting server with TESTING=true...")
    uvicorn.run(app, host="0.0.0.0", port=8000)