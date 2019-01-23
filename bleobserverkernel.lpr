program BleObserverKernel;
{$mode objfpc}{$modeswitch advancedrecords}{$H+}

uses 
{$ifdef BUILD_QEMUVPB } QEMUVersatilePB,                  {$endif}
{$ifdef BUILD_RPI     } BCM2708,BCM2835,                  {$endif}
{$ifdef BUILD_RPI2    } BCM2709,BCM2836,                  {$endif}
{$ifdef BUILD_RPI3    } BCM2710,BCM2837,LAN78XX,SMSC95XX, {$endif}
GlobalConfig,GlobalConst,GlobalTypes,Platform,Threads,SysUtils,Classes,Console,Logging,Ultibo,
Serial,DWCOTG,FileSystem,MMC,FATFS,Keyboard,bcmfw,StrUtils;

const 
 BleScanUnitsPerSecond          = 1600;
 BleScanInterval                = 0.800;
 BleScanWindow                  = 0.400;

 HCI_COMMAND_PKT             = $01;
 HCI_EVENT_PKT               = $04;
 OGF_HOST_CONTROL            = $03;
 OGF_LE_CONTROL              = $08;
 OGF_VENDOR                  = $3f;
 LL_SCAN_PASSIVE             = $00;
 LL_SCAN_ACTIVE              = $01;

 ADV_IND                     = $00; // Connectable undirected advertising(default)
 ADV_DIRECT_IND_HI           = $01; // Connectable high duty cycle directed advertising
 ADV_SCAN_IND                = $02; // Scannable undirected advertising
 ADV_NONCONN_IND             = $03; // Non connectable undirected advertising
 ADV_DIRECT_IND_LO           = $04; // Connectable low duty cycle directed advertising

 // Advertising Data Types
 ADT_FLAGS                   = $01; // Flags
 ADT_INCOMPLETE_UUID16       = $02; // Incomplete List of 16-bit Service Class UUIDs
 ADT_COMPLETE_UUID16         = $03; // Complete List of 16-bit Service Class UUIDs
 ADT_INCOMPLETE_UUID32       = $04; // Incomplete List of 32-bit Service Class UUIDs
 ADT_COMPLETE_UUID32         = $05; // Complete List of 32-bit Service Class UUIDs
 ADT_INCOMPLETE_UUID128      = $06; // Incomplete List of 128-bit Service Class UUIDs
 ADT_COMPLETE_UUDI128        = $07; // Complete List of 128-bit Service Class UUIDs
 ADT_SHORTENED_LOCAL_NAME    = $08; // Shortened Local name
 ADT_COMPLETE_LOCAL_NAME     = $09; // Complete Local name
 ADT_POWER_LEVEL             = $0A; // Tx Power Level
 ADT_DEVICE_CLASS            = $0D; // Class of Device
 ADT_SERVICE_DATA            = $16; // Service data, starts with service uuid followed by data
 ADT_DEVICE_APPEARANCE       = $19; // Device appearance
 ADT_MANUFACTURER_SPECIFIC   = $FF;

 ManufacturerApple           = $004c;
 ManufacturerEstimote        = $015d;
 ManufacturerFlic            = $030f;
 ManufacturerLogitech        = $01da;
 ManufacturerMicrosoft       = $0006;
 ManufacturerTesting         = $ffff;

 BDADDR_LEN                  = 6;

 BleBufferSize               = 64;

type 
 TArrayOfByte = Array of Byte;

var 
 Console1:TWindowHandle;
 ScanRxCount:Integer;
 BluetoothUartDeviceDescription:String;
 ScanCycleCounter:LongWord;
 ScanIdle:Boolean;
 ScanStartTime:LongWord;
 Margin:LongWord;
 ReadBackLog:Integer;
 LastDeviceStatus:LongWord;
 HciSequenceNumber:Integer = 0;
 ch:char;
 UART0:PSerialDevice = Nil;
 KeyboardLoopHandle:TThreadHandle = INVALID_HANDLE_VALUE;
 ReadByteCounter:Integer;

function ReadByte:Byte; forward;

procedure Log(S:String);
begin
 LoggingOutput(S);
end;

procedure RestoreBootFile(Prefix,FileName:String);
var 
 Source:String;
begin
 if BoardGetType <> BOARD_TYPE_QEMUVPB then
  begin
   Source:=Prefix + '-' + FileName;
   Log(Format('Restoring from %s ...',[Source]));
   while not DirectoryExists('C:\') do
    Sleep(500);
   if FileExists(Source) then
    CopyFile(PChar(Source),PChar(FileName),False);
   Log(Format('Restoring from %s done',[Source]));
  end;
end;

function ogf(op:Word):byte;
begin
 Result:=(op shr 10) and $3f;
end;

function ocf(op:Word):Word;
begin
 Result:=op and $3ff;
end;

function ErrToStr(code:byte):string;
begin
 case code of 
  $00:Result:='Success';
  $01:Result:='Unknown HCI Command';
  $02:Result:='Unknown Connection Identifier';
  $03:Result:='Hardware Failure';
  $04:Result:='Page Timeout';
  $05:Result:='Authentication Failure';
  $06:Result:='PIN or Key Missing';
  $07:Result:='Memory Capacity Exceeded';
  $08:Result:='Connection Timeout';
  $09:Result:='Connection Limit Exceeded';
  $0A:Result:='Synchronous Connection Limit To A Device Exceeded';
  $0B:Result:='ACL Connection Already Exists';
  $0C:Result:='Command Disallowed';
  $0D:Result:='Connection Rejected due to Limited Resources';
  $0E:Result:='Connection Rejected due To Security Reasons';
  $0F:Result:='Connection Rejected due to Unacceptable BD_ADDR';
  $10:Result:='Connection Accept Timeout Exceeded';
  $11:Result:='Unsupported Feature or Parameter Value';
  $12:Result:='Invalid HCI Command Parameters';
  $13:Result:='Remote User Terminated Connection';
  $14:Result:='Remote Device Terminated Connection due to Low Resources';
  $15:Result:='Remote Device Terminated Connection due to Power Off';
  $16:Result:='Connection Terminated By Local Host';
  $17:Result:='Repeated Attempts';
  $18:Result:='Pairing Not Allowed';
  $19:Result:='Unknown LMP PDU';
  $1A:Result:='Unsupported Remote Feature / Unsupported LMP Feature';
  $1B:Result:='SCO Offset Rejected';
  $1C:Result:='SCO Interval Rejected';
  $1D:Result:='SCO Air Mode Rejected';
  $1E:Result:='Invalid LMP Parameters / Invalid LL Parameters';
  $1F:Result:='Unspecified Error';
  $20:Result:='Unsupported LMP Parameter Value / Unsupported LL Parameter Value';
  $21:Result:='Role Change Not Allowed';
  $22:Result:='LMP Response Timeout / LL Response Timeout';
  $23:Result:='LMP Error Transaction Collision';
  $24:Result:='LMP PDU Not Allowed';
  $25:Result:='Encryption Mode Not Acceptable';
  $26:Result:='Link Key cannot be Changed';
  $27:Result:='Requested QoS Not Supported';
  $28:Result:='Instant Passed';
  $29:Result:='Pairing With Unit Key Not Supported';
  $2A:Result:='Different Transaction Collision';
  $2B:Result:='Reserved';
  $2C:Result:='QoS Unacceptable Parameter';
  $2D:Result:='QoS Rejected';
  $2E:Result:='Channel Classification Not Supported';
  $2F:Result:='Insufficient Security';
  $30:Result:='Parameter Out Of Mandatory Range';
  $31:Result:='Reserved';
  $32:Result:='Role Switch Pending';
  $33:Result:='Reserved';
  $34:Result:='Reserved Slot Violation';
  $35:Result:='Role Switch Failed';
  $36:Result:='Extended Inquiry Response Too Large';
  $37:Result:='Secure Simple Pairing Not Supported By Host';
  $38:Result:='Host Busy - Pairing';
  $39:Result:='Connection Rejected due to No Suitable Channel Found';
  $3A:Result:='Controller Busy';
  $3B:Result:='Unacceptable Connection Parameters';
  $3C:Result:='Directed Advertising Timeout';
  $3D:Result:='Connection Terminated due to MIC Failure';
  $3E:Result:='Connection Failed to be Established';
  $3F:Result:='MAC Connection Failed';
  $40:Result:='Coarse Clock Adjustment Rejected but Will Try to Adjust Using Clock';
 end;
end;

procedure Fail(Message:String);
begin
 raise Exception.Create(Message);
end;

procedure HciCommand(OpCode:Word; Params:array of byte);
var 
 i:integer;
 Cmd:array of byte;
 res,count:LongWord;
 PacketType,EventCode,PacketLength,CanAcceptPackets,Status:Byte;
 Acknowledged:Boolean;
begin
 Inc(HciSequenceNumber);
 // if OpCode <> $fc4c then
 //  Log(Format('hci %d op %04.4x',[HciSequenceNumber,OpCode]));
 SetLength(Cmd,length(Params) + 4);
 Cmd[0]:=HCI_COMMAND_PKT;
 Cmd[1]:=lo(OpCode);
 Cmd[2]:=hi(OpCode);
 Cmd[3]:=length(Params);
 for i:=0 to length(Params) - 1 do
  Cmd[4 + i]:=Params[i];
 count:=0;
 res:=SerialDeviceWrite(UART0,@Cmd[0],length(Cmd),SERIAL_WRITE_NONE,count);
 if res = ERROR_SUCCESS then
  begin
   Acknowledged:=False;
   while not Acknowledged do
    begin
     PacketType:=ReadByte;
     if PacketType <> HCI_EVENT_PKT then
      Fail(Format('event type not hci event: %d',[PacketType]));
     EventCode:=ReadByte;
     if EventCode = $0E then
      begin
       PacketLength:=ReadByte;
       if PacketLength <> 4 then
        Fail(Format('packet length not 4: %d',[PacketLength]));
       CanAcceptPackets:=ReadByte;
       if CanAcceptPackets <> 1 then
        Fail(Format('can accept packets not 1: %d',[CanAcceptPackets]));
       ReadByte; // completed command low
       ReadByte; // completed command high
       Status:=ReadByte;
       Acknowledged:=True;
      end
     else if EventCode = $0F then
           begin
            PacketLength:=ReadByte;
            if PacketLength <> 4 then
             Fail(Format('packet length not 4: %d',[PacketLength]));
            Status:=ReadByte;
            CanAcceptPackets:=ReadByte;
            if CanAcceptPackets <> 1 then
             Fail(Format('can accept packets not 1: %d',[CanAcceptPackets]));
            ReadByte; // completed command low
            ReadByte; // completed command high
            Acknowledged:=True;
           end
     else
      begin
       PacketLength:=ReadByte;
       Log(Format('HciCommand discarding event %d length %d',[EventCode,PacketLength]));
       for I:=1 to PacketLength do
        ReadByte;
       Sleep(5*1000);
       // Fail(Format('event code not command completed nor status: %02.2x',[EventCode]));
      end;
    end;
   if Status <> 0 then
    Fail(Format('status not 0: %d',[Status]));
  end
 else
  Log('Error writing to BT.');
end;

procedure HciCommand(OGF:byte; OCF:Word; Params:array of byte);
begin
 HciCommand((OGF shl 10) or OCF,Params);
end;

procedure Console1WriteLn(Line:string);
begin
 ConsoleWindowWrite(Console1,Line);
 ConsoleWindowClearEx(Console1,ConsoleWindowGetX(Console1),ConsoleWindowGetY(Console1),ConsoleWindowGetMaxX(Console1),ConsoleWindowGetY(Console1),False);
 ConsoleWindowWriteLn(Console1,'');
end;

procedure Console1SetForecolor(Color:LongWord);
begin
 ConsoleWindowSetForecolor(Console1,Color);
end;

procedure EndOfScanUpdateScreen;
begin
 Console1SetForecolor(COLOR_ORANGE);
 ConsoleWindowSetXY(Console1,1,1);
 Console1WriteLn(Format('%s',[BoardTypeToString(BoardGetType)]));
 Console1SetForecolor(COLOR_YELLOW);
 Console1WriteLn(Format('',[]));
 Console1WriteLn(Format('micro:bit operators detected',[]));
 Console1SetForecolor(COLOR_WHITE);
 Console1WriteLn(Format('',[]));
 Console1WriteLn(Format('screen refreshed after every ble scan interval (%5.3f seconds) - (scan window is %5.3f seconds)',[BleScanInterval,BleScanWindow]));
 ConsoleWindowClearEx(Console1,ConsoleWindowGetX(Console1),ConsoleWindowGetY(Console1),ConsoleWindowGetMaxX(Console1),ConsoleWindowGetMaxY(Console1),False);
end;

function EventReadFirstByte:Byte;
var 
 c:LongWord;
 b:Byte;
 res:Integer;
 Now:LongWord;
 EntryTime:LongWord;
begin
 Result:=0;
 EntryTime:=ClockGetCount;
 while LongWord(ClockGetCount - EntryTime) < 10*1000*1000 do
  begin
   Now:=ClockGetCount;
   c:=0;
   res:=SerialDeviceRead(UART0,@b,1,SERIAL_READ_NON_BLOCK,c);
   if (res = ERROR_SUCCESS) and (c = 1) then
    begin
     Result:=b;
     Inc(ReadByteCounter);
     if ScanIdle then
      begin
       ScanIdle:=False;
       ScanStartTime:=Now;
       ScanRxCount:=0;
       if (ScanCycleCounter >= 1) and (LongWord(Now - EntryTime) div 1000 < Margin) then
        begin
         Margin:=LongWord(Now - EntryTime) div 1000;
         LoggingOutput(Format('lowest available processing time between scans is now %5.3fs',[Margin / 1000]));
        end;
      end;
     Inc(ScanRxCount);
     exit;
    end
   else
    begin
     if (not ScanIdle) and (LongWord(Now - ScanStartTime)/(1*1000*1000)  > BleScanWindow + 0.200)  then
      begin
       ScanIdle:=True;
       Inc(ScanCycleCounter);
//     EndOfScanUpdateScreen;
      end;
     ThreadYield;
    end;
  end;
 Fail('timeout waiting for serial read byte');
end;

function ReadByte:Byte;
var 
 c:LongWord;
 b:Byte;
 res:Integer;
 EntryTime:LongWord;
 SerialStatus:LongWord;
begin
 Result:=0;
 EntryTime:=ClockGetCount;
 while LongWord(ClockGetCount - EntryTime) < 1*1000*1000 do
  begin
   c:=0;
   res:=SerialDeviceRead(UART0,@b,1,SERIAL_READ_NON_BLOCK,c);
   if (res = ERROR_SUCCESS) and (c = 1) then
    begin
     Result:=b;
     Inc(ReadByteCounter);
     res:=SerialDeviceRead(UART0,@b,1,SERIAL_READ_PEEK_BUFFER,c);
     if c > ReadBackLog then
      begin
       ReadBackLog:=c;
       LoggingOutput(Format('highest SERIAL_READ_PEEK_BUFFER is now %d',[ReadBackLog]));
      end;
     SerialStatus:=SerialDeviceStatus(UART0);
     SerialStatus:=SerialStatus and not (SERIAL_STATUS_RX_EMPTY or SERIAL_STATUS_TX_EMPTY);
     if SerialStatus <> LastDeviceStatus then
      begin
       LastDeviceStatus:=SerialStatus;
       LoggingOutput(Format('SerialDeviceStatus changed %08.8x',[SerialStatus]));
      end;
     exit;
    end
   else
    ThreadYield;
  end;
 Fail('timeout waiting for serial read byte');
end;

function IsBlueToothAvailable:Boolean;
begin
 Result:=True;
 Log(Format('Board is %s',[BoardTypeToString(BoardGetType)]));
 case BoardGetType of 
  BOARD_TYPE_RPI3B:
                   begin
                    BluetoothUartDeviceDescription:='BCM2837 PL011 UART';
                    PrepareBcmFirmware(0);
                   end;
  BOARD_TYPE_RPI3B_PLUS:
                        begin
                         BluetoothUartDeviceDescription:='BCM2837 PL011 UART';
                         PrepareBcmFirmware(1);
                        end;
  BOARD_TYPE_RPI_ZERO_W:
                        begin
                         BluetoothUartDeviceDescription:='BCM2835 PL011 UART';
                         PrepareBcmFirmware(0);
                        end;
  else
   begin
    Log('');
    Log('');
    Log('Bluetooth is not available on this board');
    Result:=False;
   end;
 end;
end;

function OpenUART0:boolean;
var 
 res:LongWord;
begin
 Result:=False;
 UART0:=SerialDeviceFindByDescription(BluetoothUartDeviceDescription);
 if UART0 = nil then
  begin
   Log('Cannot find UART0');
   exit;
  end;
 if BoardGetType = BOARD_TYPE_RPI_ZERO_W then
  res:=SerialDeviceOpen(UART0,115200,SERIAL_DATA_8BIT,SERIAL_STOP_1BIT,SERIAL_PARITY_NONE,SERIAL_FLOW_RTS_CTS,0,0)
 else
  res:=SerialDeviceOpen(UART0,115200,SERIAL_DATA_8BIT,SERIAL_STOP_1BIT,SERIAL_PARITY_NONE,SERIAL_FLOW_NONE,0,0);
 if res = ERROR_SUCCESS then
  begin
   Result:=True;
   ReadBackLog:=0;
   LastDeviceStatus:=0;

   GPIOFunctionSelect(GPIO_PIN_14,GPIO_FUNCTION_IN);
   GPIOFunctionSelect(GPIO_PIN_15,GPIO_FUNCTION_IN);

   GPIOFunctionSelect(GPIO_PIN_32,GPIO_FUNCTION_ALT3);     // TXD0
   GPIOFunctionSelect(GPIO_PIN_33,GPIO_FUNCTION_ALT3);     // RXD0
   GPIOPullSelect(GPIO_PIN_32,GPIO_PULL_NONE);             //Added
   GPIOPullSelect(GPIO_PIN_33,GPIO_PULL_UP);               //Added

   if BoardGetType = BOARD_TYPE_RPI_ZERO_W then
    begin
     GPIOFunctionSelect(GPIO_PIN_30,GPIO_FUNCTION_ALT3);     // RTS
     GPIOFunctionSelect(GPIO_PIN_31,GPIO_FUNCTION_ALT3);     // CTS
     GPIOPullSelect(GPIO_PIN_30,GPIO_PULL_UP);
     GPIOPullSelect(GPIO_PIN_31,GPIO_PULL_NONE);
    end;

   Sleep(50);
  end;
end;

procedure ResetChip;
begin
 HciCommand(OGF_HOST_CONTROL,$03,[]);
end;

procedure CloseUART0;
begin
 SerialDeviceClose(UART0);
 UART0:=Nil;
end;

procedure BCMLoadFirmware;
var 
 Params:array of byte;
 len:integer;
 Op:Word;
 Index:Integer;
 I:Integer;
 P:Pointer;
function GetByte:Byte;
begin
 Result:=PByte(P)^;
 Inc(P);
 Inc(Index);
end;
begin
 Console1WriteLn('Firmware load ...');
 Log('Firmware load ...');
 HciCommand(OGF_VENDOR,$2e,[]);
 Index:=0;
 P:=BcmFirmwarePointer;
 while Index < BcmFirmwareLength do
  begin
   Op:=GetByte;
   Op:=Op or (GetByte shl 8);
   Len:=GetByte;
   SetLength(Params,Len);
   for I:= 0 to Len - 1 do
    Params[I]:=GetByte;
   HciCommand(Op,Params);
  end;
 CloseUart0;
 Sleep(50);
 OpenUart0;
 Sleep(50);
 Console1WriteLn('Firmware load done');
 Log('Firmware load done');
end;

{$ifdef BUILD_QEMUVPB}

procedure StartLogging;
begin
 LOGGING_INCLUDE_COUNTER:=False;
 LOGGING_INCLUDE_TICKCOUNT:=True;
 SERIAL_REGISTER_LOGGING:=True;
 SerialLoggingDeviceAdd(SerialDeviceGetDefault);
 SERIAL_REGISTER_LOGGING:=False;
 LoggingDeviceSetDefault(LoggingDeviceFindByType(LOGGING_TYPE_SERIAL));
end;

{$else}

procedure StartLogging;
begin
 LOGGING_INCLUDE_COUNTER:=False;
 LOGGING_INCLUDE_TICKCOUNT:=True;
 CONSOLE_REGISTER_LOGGING:=True;
 CONSOLE_LOGGING_POSITION:=CONSOLE_POSITION_RIGHT;
 LoggingConsoleDeviceAdd(ConsoleDeviceGetDefault);
 LoggingDeviceSetDefault(LoggingDeviceFindByType(LOGGING_TYPE_CONSOLE));
end;

{$endif}

procedure SetLEScanParameters(Type_:byte;Interval,Window:Word;OwnAddressType,FilterPolicy:byte);
begin
 HciCommand(OGF_LE_CONTROL,$0b,[Type_,lo(Interval),hi(Interval),lo(Window),hi(Window),OwnAddressType,FilterPolicy]);
end;

procedure SetLEScanEnable(State,Duplicates:boolean);
var 
 Params:Array of Byte;
begin
 SetLength(Params,2);
 if State then
  Params[0]:=$01
 else
  Params[0]:=$00;
 if Duplicates then
  Params[1]:=$01
 else
  Params[1]:=$00;
 HciCommand(OGF_LE_CONTROL,$0c,Params);
end;

procedure StartPassiveScanning;
begin
 SetLEScanParameters(LL_SCAN_PASSIVE,Round(BleScanInterval*BleScanUnitsPerSecond),Round(BleScanWindow*BleScanUnitsPerSecond),$00,$00);
 SetLEScanEnable(True,False);
end;

procedure StartActiveScanning;
begin
 SetLEScanParameters(LL_SCAN_ACTIVE,Round(BleScanInterval*BleScanUnitsPerSecond),Round(BleScanWindow*BleScanUnitsPerSecond),$00,$00);
 SetLEScanEnable(True,False);
end;

procedure StopScanning;
begin
 SetLEScanEnable(False,False);
end;

// le control
procedure SetLEEventMask(Mask:QWord);
var 
 Params:array of byte;
 MaskHi,MaskLo:DWord;
begin
 MaskHi:=(Mask shr 32) and $FFFFFFFF;
 MaskLo:=Mask and $FFFFFFFF;
 SetLength(Params,8);
 Params[0]:=MaskLo and $ff;   // lsb
 Params[1]:=(MaskLo shr 8) and $ff;
 Params[2]:=(MaskLo shr 16) and $ff;
 Params[3]:=(MaskLo shr 24) and $ff;
 Params[4]:=MaskHi and $ff;   // lsb
 Params[5]:=(MaskHi shr 8) and $ff;
 Params[6]:=(MaskHi shr 16) and $ff;
 Params[7]:=(MaskHi shr 24) and $ff;
 HciCommand(OGF_LE_CONTROL,$01,Params);
end;

function KeyboardLoop(Parameter:Pointer):PtrInt;
begin
 Result:=0;
 while True do
  begin
   if ConsoleGetKey(ch,nil) then
    case uppercase(ch) of 
     #27 : SystemRestart(0);
     'Q' : SystemRestart(0);
     'R' :
          begin
           RestoreBootFile('bleobserverkernel','config.txt');
           SystemRestart(0);
          end;
    end;
  end;
end;

function MacAddressTypeToStr(MacAddressType:Byte):String;
begin
 case MacAddressType of 
  $00:Result:='P';
  $01:Result:='R';
  else
   Result:='?';
 end;
end;

function AdEventTypeToStr(AdEventType:Byte):String;
begin
 case AdEventType of 
  $00:Result:='C';
  $01:Result:='D';
  $02:Result:='S';
  $03:Result:='N';
  $04:Result:='R';
  else
   Result:='?';
 end;
end;

function AsWord(Hi,Lo:Integer):Word;
begin
 Result:=(Hi shl 8) or Lo;
end;

type
 PEventHeader = ^TEventHeader;
 TEventHeader = packed record
  EventType:Byte;
  EventSubtype:Byte;
  EventLength:Byte;
 end;

 PBleEvent = ^TBleEvent;
 TBleEvent = packed record
  LeEventType:Byte;
  AnotherByte:Byte;
 end;

 PBleBroadcast = ^TBleBroadcast;
 TBleBroadcast = packed record
  AdEventType:Byte;
  AddressType:Byte;
  Address:Array[0 .. 5] of Byte;
  Prefix:Array[0 .. 5] of Byte;
  Manufacturer:Array[0 .. 1] of Byte;
  Signature:Array[0 .. 2] of Byte;
  ReleaseNumber:Byte;
  IntendedReceiverBtMac:Array[0 .. 5] of Byte;
  Flags:Byte;
  ToggleCounterLow:Byte;
  ToggleCounterHigh:Byte;
  ToggleHistory:Array[0 .. 10] of Byte;
  Rssi:Byte;
 end;

 PButtonDevice = ^TButtonDevice;
 TButtonDevice = record
  AddressString:String;
  ProcessedToggleCounter:Word;
 end;

const
 ExpectedPrefix:Array[0 .. 5] of Byte = ($1f,$02,$01,$06,$1b,$ff);
 ExpectedManufacturer:Array[0 .. 1] of Byte = ($ff,$ff);
 ExpectedSignature:Array[0 .. 2] of Byte = ($39,$a8,$03);

 Flags_Button_A = $02;
 Flags_Button_B = $01;

var
 EventBuffer:Array[0 .. 63] of Byte;
 EventBufferLength:Integer;
 EventBufferIndex:Integer;
 ButtonDevices:Array of TButtonDevice;

function NextRecord3(var What:Pointer;Message:String;ByteCount:Integer):Boolean;
begin
// Log(Format('NextRecord %s %d bytes index %d total %d',[Message,ByteCount,EventBufferIndex,EventBufferLength]));
 Result:=False;
 if EventBufferIndex + ByteCount > EventBufferLength then
  exit;
 What:=@EventBuffer[EventBufferIndex];
 Inc(EventBufferIndex,ByteCount);
// Log(Format('NextRecord %s %d bytes leaving %d',[Message,ByteCount,EventBufferLength - EventBufferIndex]));
 Result:=True;
end;

function NextRecord4(var What:Pointer;Message:String;ByteCount:Integer):Boolean;
begin
 Log(Format('NextRecord %s %d bytes index %d total %d',[Message,ByteCount,EventBufferIndex,EventBufferLength]));
 Result:=False;
 if EventBufferIndex + ByteCount > EventBufferLength then
  exit;
 What:=@EventBuffer[EventBufferIndex];
 Inc(EventBufferIndex,ByteCount);
 Log(Format('NextRecord %s %d bytes leaving %d',[Message,ByteCount,EventBufferLength - EventBufferIndex]));
 Result:=True;
end;

function EndOfEventBuffer:Boolean;
begin
 Result:=EventBufferIndex = EventBufferLength;
end;

procedure ButtonDeviceInitialize(Device:PButtonDevice;SetAddressString:String);
begin
 with Device^ do
  begin
   AddressString:=SetAddressString;
   ProcessedToggleCounter:=0;
  end;
end;

procedure ParseBleBroadcast;
var
 I:Integer;
 S,AddressString:String;
 Match:Boolean;
 ToggleCounter:Word;
 Broadcast:PBleBroadcast;
 ButtonDevice:PButtonDevice;
 ButtonsState:Integer;
 HistoryString:String;
 Chord:String;
 procedure AddToHistoryString(X:Integer);
 var
  S:String;
  Mask,Masked:Integer;
 begin
  Mask:=Flags_Button_A or Flags_Button_B;
  Masked:=X and Mask;
  if Masked = Mask then
   S:='2'
  else if Masked = Flags_Button_A then
   S:='A'
  else if Masked = Flags_Button_B then
   S:='B'
  else
   S:='.';
  HistoryString:=HistoryString + S;
 end;
begin
 if not NextRecord3(Broadcast,'ParseBleBroadcast',SizeOf(TBleBroadcast)) then
  exit;
 if not EndOfEventBuffer then
  begin
   Log(Format('discarded chord message (too long) %d bytes remaining',[EventBufferLength - EventBufferIndex]));
   exit;
  end;
 with Broadcast^ do
  begin
   Match:=True;
   for I:=0 to High(Prefix) do
    Match:=Match and (Prefix[I] = ExpectedPrefix[I]);
   for I:=0 to High(Manufacturer) do
    Match:=Match and (Manufacturer[I] = ExpectedManufacturer[I]);
   for I:=0 to High(Signature) do
    Match:=Match and (Signature[I] = ExpectedSignature[I]);
   if Match then
    begin
     AddressString:='';
     for I:=0 to 5 do
      AddressString:=Address[I].ToHexString(2) + AddressString;
     AddressString:=AddressString + MacAddressTypeToStr(AddressType) + AdEventTypeToStr(AdEventType);
     S:='';
     for I:=0 to High(ToggleHistory) do
      begin
       if I mod 4 = 0 then
        S:=S + ' ';
       S:=S + ToggleHistory[I].ToHexString(2);
      end;
     ButtonDevice:=nil;
     for I:=0 to High(ButtonDevices) do
      if ButtonDevices[I].AddressString = AddressString then
       ButtonDevice:=@ButtonDevices[I];
     if ButtonDevice = nil then
      begin
       SetLength(ButtonDevices,Length(ButtonDevices) + 1);
       ButtonDevice:=@ButtonDevices[High(ButtonDevices)];
       ButtonDeviceInitialize(ButtonDevice,AddressString);
       Console1WriteLn('');
       Console1WriteLn(Format('%s new button device',[AddressString]));
      end;
     ToggleCounter:=AsWord(ToggleCounterHigh,ToggleCounterLow);
     if ToggleCounter <> ButtonDevice^.ProcessedToggleCounter then
      begin
       HistoryString:='';
       ButtonsState:=Flags and (Flags_Button_A or Flags_Button_B);
       AddToHistoryString(ButtonsState);
       for I:=0 to Min(SizeOf(ToggleHistory)*8,ToggleCounter) - 1 do
        begin
         if (ToggleHistory[I div 8] shr (7 - (I mod 8))) and 1= 0 then
          ButtonsState:=ButtonsState xor FLAGS_BUTTON_A
         else
          ButtonsState:=ButtonsState xor FLAGS_BUTTON_B;
         AddToHistoryString(ButtonsState);
        end;
       Console1WriteLn(Format('%s flags %02.2x toggle count %3d history %s',[AddressString,Flags,ToggleCounter,S]));
       if HistoryString <> '' then
        begin
//       Console1WriteLn(Format('          %s',[HistoryString]));
         if HistoryString[1] = '.' then
          begin
           Chord:=AnsiRightStr(HistoryString,Length(HistoryString) - 1);
           Chord:=AnsiLeftStr(Chord,AnsiPos('.',Chord) - 1);
           Chord:=AnsiReverseString(Chord);
           Console1WriteLn(Format('   Chord: %s',[Chord]));
          end
         else
          begin
           Chord:=HistoryString;
           Chord:=AnsiLeftStr(Chord,AnsiPos('.',Chord) - 1);
           Chord:=AnsiReverseString(Chord);
           Console1WriteLn(Format(' Pending: %s',[Chord]));
          end;
        end;
       ButtonDevice^.ProcessedToggleCounter:=ToggleCounter;
      end;
    end;
  end;
end;

procedure ParseBleEvent;
var
 Event:PBleEvent;
begin
 if not NextRecord3(Event,'ParseBleEvent',SizeOf(TBleEvent)) then
  exit;
 with Event^ do
  begin
   if LeEventType <> $02 then
    begin
     Log(Format('discarded ble event type %02.2x',[LeEventType]));
     exit;
    end;
   if AnotherByte <> $01 then
    begin
     Log(Format('discarded ble event type %02.2x another byte %02.2x',[LeEventType,AnotherByte]));
     exit;
    end;
   ParseBleBroadcast;
  end;
end;

procedure ParseEventHeader;
var
 Event:PEventHeader;
begin
 if not NextRecord3(Event,'ParseEventHeader',SizeOf(TEventHeader)) then
  exit;
 with Event^ do
  begin
   if EventType <> $04 then
    begin
     Log(Format('ParseEvent discarded type %02.2x subtype %02.2x length %d',[EventType,EventSubType,EventLength]));
     exit;
    end;
   if EventSubType <> $3e then
    begin
     Log(Format('ParseEvent discarded type %02.2x subtype %02.2x length %d',[EventType,EventSubType,EventLength]));
     exit;
    end;
   ParseBleEvent;
  end;
end;

procedure ReadAndParseEventHeader;
var 
 I:Integer;
 EventHeader:PEventHeader;
 S:String;
 procedure PutByte(X:Byte);
  begin
   EventBuffer[EventBufferLength]:=X;
   Inc(EventBufferLength);
   S:=S + X.ToHexString(2);
   if Length(S) mod 8 = 0 then
    S:=S + ' ';
  end;
begin
 S:='';
 EventBufferLength:=0;
 PutByte(EventReadFirstByte);
 for I:=1 to SizeOf(TEventHeader) - 1 do
  PutByte(ReadByte);
 EventHeader:=@EventBuffer;
// Log(Format('notice type %02.2x subtype %02.2x length %d',[EventHeader^.EventType,EventHeader^.EventSubtype,EventHeader^.EventLength]));
 EventBufferIndex:=0;
 if not NextRecord3(EventHeader,'ReadAndParseEventHeader',SizeOf(TEventHeader)) then
  begin
   Log(Format('Halting - EventBuffer %08.8x EventHeader %08.8x',[LongWord(@EventBuffer),LongWord(EventHeader)]));
   ThreadHalt(0);
  end;
 with EventHeader^ do
  begin
   if EventBufferLength + EventLength > SizeOf(EventBuffer) then
    begin
     for I:=1 to EventLength do
      ReadByte;
     Log(Format('discarded (too long) type %02.2x subtype %02.2x length %d',[EventType,EventSubtype,EventLength]));
     exit;
    end;
//   Log(Format('EventLength %d',[EventLength]));
   for I:=1 to EventLength do
    PutByte(ReadByte);
  end;
// Log(Format('event %s',[S]));
 EventBufferIndex:=0;
 ParseEventHeader;
end;

begin
 Console1 := ConsoleWindowCreate(ConsoleDeviceGetDefault,CONSOLE_POSITION_LEFT,False);
 ConsoleWindowSetBackcolor(Console1,COLOR_BLACK);
 ConsoleWindowSetForecolor(Console1,COLOR_YELLOW);
 ConsoleWindowClear(Console1);
// Console2 := ConsoleWindowCreate(ConsoleDeviceGetDefault,CONSOLE_POSITION_TOPRIGHT,False);
// ConsoleWindowSetBackcolor(Console2,COLOR_BLACK);
// ConsoleWindowSetForecolor(Console2,COLOR_BLUE);
// ConsoleWindowClear(Console2);

 RestoreBootFile('default','config.txt');
 StartLogging;
 Log('ble-oberserver');

 BeginThread(@KeyboardLoop,Nil,KeyboardLoopHandle,THREAD_STACK_DEFAULT_SIZE);

 // EndOfScanUpdateScreen;

 SetLength(ButtonDevices,0);

 if IsBlueToothAvailable then
  begin
   ReadByteCounter:=0;
   OpenUart0;
   ResetChip;
   try
    BCMLoadFirmware;
   except
    on E:Exception do
         begin
          LoggingOutput(Format('load exception %s',[E.Message]));
         end;
  end;
 SetLEEventMask($ff);
 Log('Init complete');
 ScanCycleCounter:=0;
 ReadByteCounter:=0;
 while True do
  begin
   ReadBackLog:=0;
   Margin:=High(Margin);
   StartActiveScanning;
   Log('Receiving scan data');
   ScanIdle:=True;
   ScanRxCount:=0;
   while True do
    ReadAndParseEventHeader;
  end;
end;
ThreadHalt(0);
end.
