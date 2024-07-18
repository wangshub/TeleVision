from vuer import Vuer, VuerSession
from vuer.schemas import Hands
from asyncio import sleep


cert_file = './cert.pem'
key_file = './key.pem'
app = Vuer(host='0.0.0.0', cert=cert_file, key=key_file, queries=dict(grid=False), queue_len=3)

@app.add_handler("HAND_MOVE")
async def handler(event, session):
    print(f"Movement Event: key-{event.key}", event.value)
    print(f"leftHand={event.value['leftHand']}")
    print(f"rightHand={event.value['rightHand']}")
    print('-'*20)
    
    
    
    

@app.spawn(start=True)
async def main(session: VuerSession):
    # Important: You need to set the `stream` option to `True` to start
    # streaming the hand movement.
    session.upsert @ Hands(fps=30, stream=True, key="hands")

    while True:
        await sleep(1)