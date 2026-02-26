import asyncio
import websockets
import json

async def echo(websocket):
    print("✅ Spring Boot가 연결되었습니다!")
    try:
        async for message in websocket:
            print("\n[메시지 수신됨]")
            data = json.loads(message)
            print(json.dumps(data, indent=4, ensure_ascii=False))
            
            # (옵션) 진짜 rosbridge처럼 응답을 보내줄 수도 있음
            # await websocket.send("잘 받았다 오바")
    except websockets.exceptions.ConnectionClosed:
        print("❌ 연결이 끊어졌습니다.")

async def main():
    print("🤖 가짜 ROS Bridge 서버 시작 (ws://localhost:9090)")
    # localhost, 9090 포트로 서버 열기
    async with websockets.serve(echo, "localhost", 9090):
        await asyncio.Future()  # 영원히 실행

if __name__ == "__main__":
    asyncio.run(main())