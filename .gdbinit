target ext :3333
monitor arm semihosting enable

def update
  make
  load
  mon reset
end
