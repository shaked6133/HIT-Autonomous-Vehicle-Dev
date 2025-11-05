from fastapi import FastAPI

app = FastAPI(
    title="Autonomous Vehicle API",
    description="FastAPI backend for the Intelligent Autonomous Vehicle course",
    version="1.0.0",
)

@app.get("/")
async def root():
    return {"message": "Welcome to the Intelligent Autonomous Vehicle API 🚗🤖"}
