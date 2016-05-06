MODULE LOGGER

!////////////////
!GLOBAL VARIABLES/
!////////////////
!PC communication
VAR socketdev clientSocket;
VAR socketdev serverSocket;
PERS num loggerPort:= 5001;

!Robot configuration	
!VAR tooldata currentTool;    
!VAR wobjdata currentWobj;
!VAR speeddata currentSpeed;
!VAR zonedata currentZone;

!//Logger sampling rate
PERS num loggerWaitTime:= 0.02;  !Recommended for real controller
!PERS num loggerWaitTime:= 0.1;    !Recommended for virtual controller
!PERS bool joints_notcartesians:= TRUE;

PROC ServerCreateAndConnect(string ip, num port)
	VAR string clientIP;
	
	SocketCreate serverSocket;
	SocketBind serverSocket, ip, port;
	SocketListen serverSocket;
	TPWrite "LOGGER: Logger waiting for incomming connections ...";
	WHILE SocketGetStatus(clientSocket) <> SOCKET_CONNECTED DO
		SocketAccept serverSocket,clientSocket \ClientAddress:=clientIP \Time:=WAIT_MAX;
		IF SocketGetStatus(clientSocket) <> SOCKET_CONNECTED THEN
			TPWrite "LOGGER: Problem serving an incomming connection.";
			TPWrite "LOGGER: Try reconnecting.";
		ENDIF
		 !Wait 0.5 seconds for the next reconnection
		 WaitTime 0.5;
	ENDWHILE
	TPWrite "LOGGER: Connected to IP " + clientIP;
ENDPROC

PROC main()
	VAR string data;
	VAR string data2;
	VAR robtarget position;
	VAR jointtarget joints;
    VAR string sendString;
	VAR bool connected;
	VAR rawbytes raw_data;
	VAR rawbytes buffer;

	VAR string date;
	VAR string time;
	VAR clock timer;

	date:= CDate();
	time:= CTime();
    ClkStart timer;
	
	connected:=FALSE;
	WaitTime 1;
	ServerCreateAndConnect ipController,loggerPort;	
	connected:=TRUE;
	WHILE TRUE DO
			!Joint Coordinates
			joints := CJointT();
			PackRawBytes joints.robax.rax_1, raw_data, 1, \Float4;
			PackRawBytes joints.robax.rax_2, raw_data, 5, \Float4;
			PackRawBytes joints.robax.rax_3, raw_data, 9, \Float4;
			PackRawBytes joints.robax.rax_4, raw_data, 13, \Float4;
			PackRawBytes joints.robax.rax_5, raw_data, 17, \Float4;
			PackRawBytes joints.robax.rax_6, raw_data, 21, \Float4;
			!data := "";
			!data := data + date + " " + time + " ";
			!data := data + NumToStr(ClkRead(timer),2) + " ";
			!data := data + NumToStr(joints.robax.rax_1,2) + " ";
			!data := data + NumToStr(joints.robax.rax_2,2) + " ";
			!data := data + NumToStr(joints.robax.rax_3,2) + " ";
			!data := data + NumToStr(joints.robax.rax_4,2) + " ";
			!data := data + NumToStr(joints.robax.rax_5,2) + " ";
			!data := data + NumToStr(joints.robax.rax_6,2) + " ";
			IF IsMechUnitActive(STN1) THEN
				PackRawBytes joints.extax.eax_b, raw_data, 25, \Float4;
				PackRawBytes joints.extax.eax_c, raw_data, 29, \Float4;
				!data := data + NumToStr(joints.extax.eax_b,2) + " ";
				!data := data + NumToStr(joints.extax.eax_c,2) + " ";!End of string
			ENDIF
			IF connected = TRUE THEN
				PackRawBytes RawBytesLen(raw_data), buffer, 1, \IntX := UDINT; ! Packet length (excluding this prefix) 
				CopyRawBytes raw_data, 1, buffer, 5; ! Message data 
				SocketSend clientSocket \RawData:=buffer;
			ENDIF
			WaitTime loggerWaitTime;
	ENDWHILE
	ERROR
	IF ERRNO=ERR_SOCK_CLOSED THEN
		TPWrite "LOGGER: Client has closed connection.";
	ELSE
		TPWrite "LOGGER: Connection lost: Unknown problem.";
		TPWrite "LOGGER: ERRNO: " + NumtoStr(ERRNO,0);
	ENDIF
	connected:=FALSE;
	!Closing the server
	SocketClose clientSocket;
	SocketClose serverSocket;
	!Reinitiate the server
	ServerCreateAndConnect ipController,loggerPort;
	connected:= TRUE;
	RETRY;
ENDPROC

ENDMODULE
