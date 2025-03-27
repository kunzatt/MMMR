import asyncio
import websockets
import json

clients = {}

# 클라이언트와 메시지를 주고받는 함수
async def handle_client(websocket, path):
    try:
        async for message in websocket:
            data = json.loads(message)
            print(f"Received: {message}")
            
            if data.get("type") == "register":
                client_type = data.get("client_type", "unknown_client")
                clients[client_type] = websocket
                print(f"Registered: {client_type}")
                response = json.dumps({"type": "ack", "message": "registered"})
                await websocket.send(response)
            elif data.get("type") == "send":
                response = json.dumps({"type": "control", "device": data.get("device"), "state": data.get("state")})
                for client_type in clients.keys():
                    if client_type == "receiver":
                        await clients[client_type].send(response)
    except websockets.exceptions.ConnectionClosed:
        print("Client disconnected")

# 서버 실행
async def main():
    server = await websockets.serve(handle_client, "0.0.0.0", 12345)
    print("WebSocket Server started on ws://localhost:12345")
    await server.wait_closed()

if __name__ == "__main__":
    asyncio.run(main())