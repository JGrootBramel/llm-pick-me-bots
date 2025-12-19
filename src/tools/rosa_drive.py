import os
from langchain_openai import ChatOpenAI
from rosa import ROSA

llm = ChatOpenAI(
    model="gpt-4o",   # RO SA wiki recommends gpt-4o as a default
    temperature=0,
    api_key=os.environ["OPENAI_API_KEY"],
)

agent = ROSA(ros_version=1, llm=llm)

print(agent.invoke("List the topics related to robot motion (cmd_vel, twist)."))
print(agent.invoke(
    "Publish to /cmd_vel (geometry_msgs/Twist): drive forward at 0.2 m/s for 2 seconds, then publish zero velocities to stop."
))