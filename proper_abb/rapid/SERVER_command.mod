MODULE SERVER_command

!////////////////
!GLOBAL VARIABLES
!////////////////

!//PC communication
VAR socketdev clientSocket;
VAR socketdev serverSocket;
VAR num instructionCode;
VAR num params{10};
VAR num nParams;

!//Correct Instruction Execution and possible return values
VAR num ok;
CONST num SERVER_BAD_MSG :=  0;
CONST num SERVER_OK := 1;

!////////////////
!LOCAL METHODS
!////////////////

!//Method to parse the message received from a PC
!// If correct message, loads values on:
!// - instructionCode.
!// - nParams: Number of received parameters.
!// - params{nParams}: Vector of received params.
PROC ParseMsg(string msg)
    !//Local variables
    VAR bool auxOk;
    VAR num ind:=1;
    VAR num newInd;
    VAR num length;
    VAR num indParam:=1;
    VAR string subString;
    VAR bool end := FALSE;

    !//Find the end character
    length := StrMatch(msg,1,"#");
    IF length > StrLen(msg) THEN
        !//Corrupt message
        nParams := -1;
    ELSE
        !//Read Instruction code
        newInd := StrMatch(msg,ind," ") + 1;
        subString := StrPart(msg,ind,newInd - ind - 1);
        auxOk:= StrToVal(subString, instructionCode);
        IF auxOk = FALSE THEN
            !//Impossible to read instruction code
            nParams := -1;
        ELSE
            ind := newInd;
            !//Read all instruction parameters (maximum of 8)
            WHILE end = FALSE DO
                newInd := StrMatch(msg,ind," ") + 1;
                IF newInd > length THEN
                    end := TRUE;
                ELSE
                    subString := StrPart(msg,ind,newInd - ind - 1);
                    auxOk := StrToVal(subString, params{indParam});
                    indParam := indParam + 1;
                    ind := newInd;
                ENDIF
            ENDWHILE
            nParams:= indParam - 1;
        ENDIF
    ENDIF
ENDPROC


!//Handshake between server and client:
!// - Creates socket.
!// - Waits for incoming TCP connection.
PROC ServerCreateAndConnect(string ip, num port)
    VAR string clientIP;

    SocketCreate serverSocket;
    SocketBind serverSocket, ip, port;
    SocketListen serverSocket;
    TPWrite "SERVER: Server waiting for incoming connections ...";
    WHILE SocketGetStatus(clientSocket) <> SOCKET_CONNECTED DO
        SocketAccept serverSocket,clientSocket \ClientAddress:=clientIP \Time:=WAIT_MAX;
        IF SocketGetStatus(clientSocket) <> SOCKET_CONNECTED THEN
            TPWrite "SERVER: Problem serving an incoming connection.";
            TPWrite "SERVER: Try reconnecting.";
        ENDIF
        !//Wait 0.5 seconds for the next reconnection
        WaitTime 0.5;
    ENDWHILE
    TPWrite "SERVER: Connected to IP " + clientIP;
ENDPROC



!///////////////////////////
!//SERVER: Main procedure //
!///////////////////////////
PROC main()
    !//Local variables
    VAR string receivedString;   !//Received string
    VAR string sendString;       !//Reply string
    VAR string addString;        !//String to add to the reply.
    VAR bool connected;          !//Client connected
    VAR bool reconnected;        !//Drop and reconnection happened during serving a command
    VAR robtarget cartesianPose;
    VAR jointtarget jointsPose;
	VAR robtarget singleCartesianTarget;

    !//Socket connection
    connected:=FALSE;
    ServerCreateAndConnect ipController,serverPort;
    connected:=TRUE;
	cancel_motion := FALSE;

    !//Server Loop
    WHILE TRUE DO
        !//Initialization of program flow variables
        ok:=SERVER_OK;              !//Correctness of executed instruction.
        reconnected:=FALSE;         !//Has communication dropped after receiving a command?
        addString := "";

        !//Wait for a command
        SocketReceive clientSocket \Str:=receivedString \Time:=WAIT_MAX;
        ParseMsg receivedString;

        !//Execution of the command
        TEST instructionCode
            CASE 0: !Ping
                IF nParams = 0 THEN
                    ok := SERVER_OK;
                ELSE
                    ok := SERVER_BAD_MSG;
                ENDIF

            CASE 1: !Cartesian Move
                IF nParams = 7 THEN
                    ok := SERVER_OK;
					          WaitUntil NOT ((n_cartesian_motion - n_cartesian_command) = 1 OR (n_cartesian_motion - n_cartesian_command) = -48);
                    command_type{n_cartesian_command} := 1;
                    cartesianTarget{n_cartesian_command} := [[params{1},params{2},params{3}],
                                       [params{4},params{5},params{6},params{7}],
                                       [0,0,0,0],
                                       externalAxis];
                    cartesian_speed{n_cartesian_command} := currentSpeed;
                    cartesianTriggSet{n_cartesian_command} := FALSE;
                    n_cartesian_command := n_cartesian_command + 1;
                    IF n_cartesian_command > 49
                      n_cartesian_command := 1;
                    !moveCompleted := FALSE;
                    !MoveL cartesianTarget, currentSpeed, currentZone, currentTool \WObj:=currentWobj ;
                    !moveCompleted := TRUE;
                ELSE
                    ok := SERVER_BAD_MSG;
                ENDIF

            CASE 2: !Joint Move
                IF nParams = 6 THEN
                    !jointsTarget:=[[params{1},params{2},params{3},params{4},params{5},params{6}], externalAxis];
                    ok := SERVER_OK;
                    !moveCompleted := FALSE;
                    !MoveAbsJ jointsTarget, currentSpeed, currentZone, currentTool \Wobj:=currentWobj;
                    !moveCompleted := TRUE;
                    TPWrite "Command 2 not implemented.";
                ELSE
                    ok :=SERVER_BAD_MSG;
                ENDIF

            CASE 3: !Get Cartesian Coordinates (with current tool and workobject)
                IF nParams = 0 THEN
                    cartesianPose := CRobT(\Tool:=currentTool \WObj:=currentWObj);
                    addString := NumToStr(cartesianPose.trans.x,2) + " ";
                    addString := addString + NumToStr(cartesianPose.trans.y,2) + " ";
                    addString := addString + NumToStr(cartesianPose.trans.z,2) + " ";
                    addString := addString + NumToStr(cartesianPose.rot.q1,3) + " ";
                    addString := addString + NumToStr(cartesianPose.rot.q2,3) + " ";
                    addString := addString + NumToStr(cartesianPose.rot.q3,3) + " ";
                    addString := addString + NumToStr(cartesianPose.rot.q4,3); !End of string
                    ok := SERVER_OK;
                ELSE
                    ok :=SERVER_BAD_MSG;
                ENDIF

            CASE 4: !Get Joint Coordinates
                IF nParams = 0 THEN
                    jointsPose := CJointT();
                    addString := NumToStr(jointsPose.robax.rax_1,5) + " ";
                    addString := addString + NumToStr(jointsPose.robax.rax_2,5) + " ";
                    addString := addString + NumToStr(jointsPose.robax.rax_3,5) + " ";
                    addString := addString + NumToStr(jointsPose.robax.rax_4,5) + " ";
                    addString := addString + NumToStr(jointsPose.robax.rax_5,5) + " ";
                    addString := addString + NumToStr(jointsPose.robax.rax_6,5); !End of string
                    ok := SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 6: !Set Tool
                IF nParams = 7 THEN
		   			        WHILE (frameMutex) DO
		        		        WaitTime .01; !// If the frame is being used by logger, wait here
		   			        ENDWHILE
					        frameMutex:= TRUE;
                    currentTool.tframe.trans.x:=params{1};
                    currentTool.tframe.trans.y:=params{2};
                    currentTool.tframe.trans.z:=params{3};
                    currentTool.tframe.rot.q1:=params{4};
                    currentTool.tframe.rot.q2:=params{5};
                    currentTool.tframe.rot.q3:=params{6};
                    currentTool.tframe.rot.q4:=params{7};
                    ok := SERVER_OK;
		    		      frameMutex:= FALSE;
					        TPWrite "Tool set";
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 7: !Set Work Object
                IF nParams = 7 THEN
                    currentWobj.oframe.trans.x:=params{1};
                    currentWobj.oframe.trans.y:=params{2};
                    currentWobj.oframe.trans.z:=params{3};
                    currentWobj.oframe.rot.q1:=params{4};
                    currentWobj.oframe.rot.q2:=params{5};
                    currentWobj.oframe.rot.q3:=params{6};
                    currentWobj.oframe.rot.q4:=params{7};
                    ok := SERVER_OK;
					        TPWrite "Work Object Set";
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 8: !Set Speed of the Robot
                IF nParams = 4 THEN
                    currentSpeed.v_tcp:=params{1};
                    currentSpeed.v_ori:=params{2};
                    currentSpeed.v_leax:=params{3};
                    currentSpeed.v_reax:=params{4};
					          TPWrite "Speed Set: ", \Num:=currentSpeed.v_tcp;
                    ok := SERVER_OK;
                ELSEIF nParams = 2 THEN
					           currentSpeed.v_tcp:=params{1};
					           currentSpeed.v_ori:=params{2};
					           TPWrite "Speed Set: ", \Num:=currentSpeed.v_tcp;
					          ok := SERVER_OK;
				        ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 9: !Set zone data
                IF nParams = 4 THEN
                    IF params{1}=1 THEN
                        currentZone.finep := TRUE;
                        currentZone.pzone_tcp := 0.0;
                        currentZone.pzone_ori := 0.0;
                        currentZone.zone_ori := 0.0;
                    ELSE
                        currentZone.finep := FALSE;
                        currentZone.pzone_tcp := params{2};
                        currentZone.pzone_ori := params{3};
                        currentZone.zone_ori := params{4};
                    ENDIF
                    ok := SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

			      CASE 10: !Joint Move to Pos
                IF nParams = 7 THEN
                    ok := SERVER_OK;
					          WaitUntil NOT ((n_cartesian_motion - n_cartesian_command) = 1 OR (n_cartesian_motion - n_cartesian_command) = -48);
                    command_type{n_cartesian_command} := 10;
                    cartesianTarget{n_cartesian_command} := [[params{1},params{2},params{3}],
                                       [params{4},params{5},params{6},params{7}],
                                       [0,0,0,0],
                                       externalAxis];
                    cartesian_speed{n_cartesian_command} := currentSpeed;
                    cartesianTriggSet{n_cartesian_command} := FALSE;
                    n_cartesian_command := n_cartesian_command + 1;
                    IF n_cartesian_command > 49
                      n_cartesian_command := 1;
                    !moveCompleted := FALSE;
                    !MoveJ cartesianTarget, currentSpeed, currentZone, currentTool \WObj:=currentWobj ;
                    !moveCompleted := TRUE;
                ELSE
                    ok := SERVER_BAD_MSG;
                ENDIF

            CASE 11: !Trigger Move linear
                IF nParams = 8 THEN
                    ok := SERVER_OK;
          					WaitUntil NOT ((n_cartesian_motion - n_cartesian_command) = 1 OR (n_cartesian_motion - n_cartesian_command) = -48);
                    IF params{8} < 1 THEN
                      command_type{n_cartesian_command} := 110;
                    ELSE
                      command_type{n_cartesian_command} := 111;
										ENDIF
                    cartesianTarget{n_cartesian_command} := [[params{1},params{2},params{3}],
                                       [params{4},params{5},params{6},params{7}],
                                       [0,0,0,0],
                                       externalAxis];
                    cartesian_speed{n_cartesian_command} := currentSpeed;
                    cartesianTriggSet{n_cartesian_command} := FALSE;
                    !TPWrite "valor "\Num:=params{8};
                    n_cartesian_command := n_cartesian_command + 1;
                    IF n_cartesian_command > 49
                      n_cartesian_command := 1;
                    !moveCompleted := FALSE;
										!IF params{8} < 1 THEN
                    !	TriggL cartesianTarget, currentSpeed, laserOFF, currentZone, currentTool \WObj:=currentWobj ;
										!ELSE
										!	TriggL cartesianTarget, currentSpeed, laserON, currentZone, currentTool \WObj:=currentWobj ;
										!ENDIF
                    !moveCompleted := TRUE;
                ELSE
                    ok := SERVER_BAD_MSG;
                ENDIF

              CASE 12: !Move external axis
        				IF nParams = 3 THEN
        					ok := SERVER_OK;
        					WaitUntil NOT ((n_cartesian_motion - n_cartesian_command) = 1 OR (n_cartesian_motion - n_cartesian_command) = -48);
        					currentSpeed.v_reax := params{3};
        					cartesian_speed{n_cartesian_command} := currentSpeed;
        					IF params{1} = 1 THEN
        					  command_type{n_cartesian_command} := 121;
        					  extAxisMove{n_cartesian_command} := params{2};
        					ENDIF
        					IF params{1} = 2 THEN
					          command_type{n_cartesian_command} := 122;
						        extAxisMove{n_cartesian_command} := params{2};
        					ENDIF
        					n_cartesian_command := n_cartesian_command + 1;
                  IF n_cartesian_command > 49
                    n_cartesian_command := 1;
                  ELSE
                    ok := SERVER_BAD_MSG;
                ENDIF

            CASE 30: !Add Cartesian Coordinates to buffer as trigger or move
                IF nParams = 7 OR nParams = 8 THEN
                    singleCartesianTarget :=[[params{1},params{2},params{3}],
                                        [params{4},params{5},params{6},params{7}],
                                        [0,0,0,0],
                                        externalAxis];
                    IF BUFFER_POS < MAX_BUFFER THEN
                        WaitTestAndSet trajectory_lock;
                        BUFFER_POS := BUFFER_POS + 1;
                        bufferTargets{BUFFER_POS} := singleCartesianTarget;
                        bufferSpeeds{BUFFER_POS} := currentSpeed;
						            bufferZones{BUFFER_POS} := currentZone;
                        IF nParams = 8 THEN
                          bufferTrigg{BUFFER_POS} := TRUE;
                          bufferTriggSet{BUFFER_POS} := params{8} > 0;
                        ELSE
                          bufferTrigg{BUFFER_POS} := FALSE;
                          bufferTriggSet{BUFFER_POS} := FALSE;
                        ENDIF
						            TPWrite "Added pose to buffer, N: ", \Num:=BUFFER_POS;
                        trajectory_lock := FALSE;
                    ENDIF
                    ok := SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 31: !Clear Cartesian Buffer
                IF nParams = 0 THEN
                  WaitTestAndSet trajectory_lock;
                    BUFFER_POS := 0;
                  trajectory_lock := FALSE;
                    ok := SERVER_OK;
					          TPWrite "Buffer clear";
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 32: !Get Buffer Size)
                IF nParams = 0 THEN
                    addString := NumToStr(BUFFER_POS,2);
                    ok := SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 33: !Execute moves in cartesianBuffer as linear moves or trigger
                IF nParams = 0 THEN
					        TPWrite "buffer size: ", \Num:=BUFFER_POS;
                    new_trajectory := true;
                    ok := SERVER_OK;
                ELSE
                    ok:=SERVER_BAD_MSG;
                ENDIF

            CASE 93: !Wait for a digital input
              IF nParams = 2 THEN
                !TODO:Seleccionar o tipo de entrada
                !TPWrite "Digital output WaitDI =", \Num:=params{1};
                TEST params{1}
                  CASE 0:
                    !WaitDI Di_FL_EstadBy,params{2};
                    WaitUntil NOT ((n_cartesian_motion - n_cartesian_command) = 1 OR (n_cartesian_motion - n_cartesian_command) = -48);
                    command_type{n_cartesian_command} := 930;
                    commandSetDO{n_cartesian_command} := params{2} <> 0;
                    n_cartesian_command := n_cartesian_command + 1;
                    IF n_cartesian_command > 49
                      n_cartesian_command := 1;
                  CASE 1:
                    !WaitDI Di_FL_ErrorLaserApagado,params{2};
                    WaitUntil NOT ((n_cartesian_motion - n_cartesian_command) = 1 OR (n_cartesian_motion - n_cartesian_command) = -48);
                    command_type{n_cartesian_command} := 931;
                    commandSetDO{n_cartesian_command} := params{2} <> 0;
                    n_cartesian_command := n_cartesian_command + 1;
                    IF n_cartesian_command > 49
                      n_cartesian_command := 1;
                  DEFAULT:
                    TPWrite "SERVER: Illegal wait code DI =", \Num:=params{1};
                    ok := SERVER_BAD_MSG;
                ENDTEST
              ELSE
                ok :=SERVER_BAD_MSG;
              ENDIF

            CASE 94: !Wait time between moves
              IF nParams = 1 THEN
              WaitUntil NOT ((n_cartesian_motion - n_cartesian_command) = 1 OR (n_cartesian_motion - n_cartesian_command) = -48);
              command_type{n_cartesian_command} := 94;
              commandWaitTime{n_cartesian_command} := params{1};
              n_cartesian_command := n_cartesian_command + 1;
              IF n_cartesian_command > 49
                n_cartesian_command := 1;
              ENDIF

      			CASE 95: !Value to GO
      				IF nParams = 2 THEN
                      !TODO:Seleccionar o tipo de entrada
          					TEST params{1}
          						CASE 0:
      								IF params{2} > 31
      									params{2} := 31;
                        				SetGO GO_FL_Programa, params{2};
      							  CASE 1:
                        				SetGO GO_FL_PotenciaLaser1, params{2};

          						DEFAULT:
                        				TPWrite "SERVER: Illegal output code GO =", \Num:=params{1};
                        				ok := SERVER_BAD_MSG;
      					    ENDTEST
              ELSE
                ok :=SERVER_BAD_MSG;
              ENDIF

			      CASE 96: !Set an analog output
                IF nParams = 2 THEN
                !TODO:Seleccionar o tipo de entrada
        					TEST params{1}
        						CASE 0:
                      				SetAO AoGTV_ExternDisk, params{2};
        						CASE 1:
        							        SetAO AoGTV_ExternMassflow, params{2};
        						DEFAULT:
                      				TPWrite "SERVER: Illegal output code AO =", \Num:=params{1};
                      ok := SERVER_BAD_MSG;
    					    ENDTEST
                ELSE
                    ok :=SERVER_BAD_MSG;
                ENDIF

    		    CASE 97: !Set or reset a digital output
              IF nParams = 2 THEN
      					!TODO:Seleccionar o tipo de entrada
                !TPWrite "Digital output SetDO =", \Num:=params{1};
      					TEST params{1}
      						CASE 0:
                    !SetDO doGTV_StartExtern, params{2};
                    WaitUntil NOT ((n_cartesian_motion - n_cartesian_command) = 1 OR (n_cartesian_motion - n_cartesian_command) = -48);
                    command_type{n_cartesian_command} := 970;
                    commandSetDO{n_cartesian_command} := params{2} <> 0;
                    n_cartesian_command := n_cartesian_command + 1;
                    IF n_cartesian_command > 49
                      n_cartesian_command := 1;
                  CASE 1:
                    !SetDO doGTV_Stop, params{2};
                    WaitUntil NOT ((n_cartesian_motion - n_cartesian_command) = 1 OR (n_cartesian_motion - n_cartesian_command) = -48);
                    command_type{n_cartesian_command} := 971;
                    commandSetDO{n_cartesian_command} := params{2} <> 0;
                    n_cartesian_command := n_cartesian_command + 1;
                    IF n_cartesian_command > 49
                      n_cartesian_command := 1;
                  CASE 2:
                    !SetDO Do_FL_RedENC, params{2};
                    WaitUntil NOT ((n_cartesian_motion - n_cartesian_command) = 1 OR (n_cartesian_motion - n_cartesian_command) = -48);
                    command_type{n_cartesian_command} := 972;
                    commandSetDO{n_cartesian_command} := params{2} <> 0;
                    n_cartesian_command := n_cartesian_command + 1;
                    IF n_cartesian_command > 49
                      n_cartesian_command := 1;
                  CASE 3:
                    !SetDO Do_FL_StandByEnc, params{2};
                    WaitUntil NOT ((n_cartesian_motion - n_cartesian_command) = 1 OR (n_cartesian_motion - n_cartesian_command) = -48);
                    command_type{n_cartesian_command} := 973;
                    commandSetDO{n_cartesian_command} := params{2} <> 0;
                    n_cartesian_command := n_cartesian_command + 1;
                    IF n_cartesian_command > 49
                      n_cartesian_command := 1;
                  CASE 4:
                    !SetDO DoWeldGas, params{2};
                    WaitUntil NOT ((n_cartesian_motion - n_cartesian_command) = 1 OR (n_cartesian_motion - n_cartesian_command) = -48);
                    command_type{n_cartesian_command} := 974;
                    commandSetDO{n_cartesian_command} := params{2} <> 0;
                    n_cartesian_command := n_cartesian_command + 1;
                    IF n_cartesian_command > 49
                      n_cartesian_command := 1;
                  CASE 5:
                    !SetDO DoRootGas, params{2};
                    WaitUntil NOT ((n_cartesian_motion - n_cartesian_command) = 1 OR (n_cartesian_motion - n_cartesian_command) = -48);
                    command_type{n_cartesian_command} := 975;
                    commandSetDO{n_cartesian_command} := params{2} <> 0;
                    n_cartesian_command := n_cartesian_command + 1;
                    IF n_cartesian_command > 49
                      n_cartesian_command := 1;
      						DEFAULT:
                		TPWrite "SERVER: Illegal output code DO =", \Num:=params{1};
                		ok := SERVER_BAD_MSG;
      					ENDTEST
              ELSE
                ok :=SERVER_BAD_MSG;
              ENDIF

            CASE 98: !returns current robot info: serial number, robotware version, and robot type
                IF nParams = 0 THEN
                    addString := GetSysInfo(\SerialNo) + "*";
                    addString := addString + GetSysInfo(\SWVersion) + "*";
                    addString := addString + GetSysInfo(\RobotType);
                    ok := SERVER_OK;
                ELSE
                    ok :=SERVER_BAD_MSG;
                ENDIF

            CASE 99: !Close Connection
                IF nParams = 0 THEN
                    TPWrite "SERVER: Client has closed connection.";
                    connected := FALSE;
                    !//Closing the server
                    SocketClose clientSocket;
                    SocketClose serverSocket;

                    !Reinitiate the server
                    ServerCreateAndConnect ipController,serverPort;
                    connected := TRUE;
                    reconnected := TRUE;
                    ok := SERVER_OK;
                ELSE
                    ok := SERVER_BAD_MSG;
                ENDIF
      			CASE 100: !Stop
      				IF nParams = 0 THEN
      					IF cancel_motion = TRUE
      						cancel_motion := FALSE;
      					TPWrite "Cancel command";
      					n_cartesian_command := n_cartesian_motion;
      					cancel_motion := TRUE;
                          ok := SERVER_OK;
                      ELSE
                          ok := SERVER_BAD_MSG;
                      ENDIF
                  DEFAULT:
                      TPWrite "SERVER: Illegal instruction code";
                      ok := SERVER_BAD_MSG;
        ENDTEST

        !Compose the acknowledge string to send back to the client
        IF connected = TRUE THEN
            IF reconnected = FALSE THEN
			         IF SocketGetStatus(clientSocket) = SOCKET_CONNECTED THEN
				           sendString := NumToStr(instructionCode,0);
                    sendString := sendString + " " + NumToStr(ok,0);
                    sendString := sendString + " " + addString;
                    SocketSend clientSocket \Str:=sendString;
			        ENDIF
            ENDIF
        ENDIF
    ENDWHILE

ERROR (LONG_JMP_ALL_ERR)
    TPWrite "SERVER: ------";
    TPWrite "SERVER: Error Handler:" + NumtoStr(ERRNO,0);
    TEST ERRNO
        CASE ERR_SOCK_CLOSED:
            TPWrite "SERVER: Lost connection to the client.";
            TPWrite "SERVER: Closing socket and restarting.";
            TPWrite "SERVER: ------";
            connected:=FALSE;
            !//Closing the server
            SocketClose clientSocket;
            SocketClose serverSocket;
            !//Reinitiate the server
            ServerCreateAndConnect ipController,serverPort;
            reconnected:= TRUE;
            connected:= TRUE;
            RETRY;
	CASE ERR_NORUNUNIT:
            TPWrite "SERVER: No contact with unit.";
            TRYNEXT;
        DEFAULT:
            TPWrite "SERVER: Unknown error.";
            TPWrite "SERVER: Closing socket and restarting.";
            TPWrite "SERVER: ------";
            connected:=FALSE;
            !//Closing the server
            SocketClose clientSocket;
            SocketClose serverSocket;
            !//Reinitiate the server
            ServerCreateAndConnect ipController,serverPort;
            reconnected:= TRUE;
            connected:= TRUE;
            RETRY;
    ENDTEST
ENDPROC

ENDMODULE
