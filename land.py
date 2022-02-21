from tello import Tello

if __name__ == "__main__":
    tello = Tello('', 8889)  
    tello.send_command('command')
    tello.send_command('land') 