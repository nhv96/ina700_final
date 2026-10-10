MODULE MainModule
    RECORD Mytargetpost
        num x;
        num y;
    ENDRECORD
    
	TASK PERS tooldata sucker:=[TRUE,[[-82.5255,33.838,171.025],[1,0,0,0]],[1.1,[20,20,25],[1,0,0,0],0,0,0]];
	TASK PERS wobjdata wobj1:=[FALSE,TRUE,"",[[844.518,-343.337,481.049],[0.999981,0.00119971,0.00155025,0.00576057]],[[0,0,0],[1,0,0,0]]];
	TASK PERS wobjdata wobj2:=[FALSE,TRUE,"",[[763.38,162.742,490.713],[0.99949,0.00316899,-0.0026938,0.0316706]],[[0,0,0],[1,0,0,0]]];
	CONST robtarget PHOME:=[[800.03,-177.70,813.24],[0.0162618,-0.228551,-0.973383,-0.00511577],[-1,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
	CONST robtarget PHEXAGON:=[[47.38,40.47,4.69],[0.0130504,-0.203534,-0.97898,-0.00153884],[-1,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
	CONST robtarget PSTAR:=[[109.17,88.68,3.04],[0.0130595,-0.203509,-0.978985,-0.00153651],[-1,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
	CONST robtarget PTRIANGLE:=[[161.16,26.93,3.92],[0.0130766,-0.203477,-0.978991,-0.00154229],[-1,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
	CONST robtarget PSQUARE:=[[170.49,144.42,2.84],[0.0130853,-0.203484,-0.97899,-0.00155356],[-1,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
	CONST robtarget PAPPROAC:=[[111.28,94.22,200.93],[0.0130856,-0.203488,-0.978989,-0.00154192],[-1,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
	
    ! Input from the Rasp must be in this format: x,y,shape,angle
    
    PROC main()
        
        VAR num selected_btn;
        VAR string input;
        VAR bool success;
        VAR string stopFlag := "stop";
        VAR Mytargetpost targetpost;
        VAR num comma_pos;

        RobotAsClientConnect;

        TPErase;

        WaitDI diOkButton, 1;
        
        WHILE input <> stopFlag DO
            input := RobotClientReciveMessage();
            comma_pos := StrFind(input, 1, ",");
            IF StrLen(input) > 0 AND comma_pos <= StrLen(input) THEN
                targetpost := processinput(input);

                TPWrite "X: " + ValToStr(targetpost.x) + " Y: " + ValToStr(targetpost.y);
                
                pickandplace2(targetpost.x), (targetpost.y);
            ENDIF
        ENDWHILE
        
        IF input = stopFlag THEN
            RobotClientCloseAndDisconnect;
        ENDIF
	ENDPROC
    
    FUNC Mytargetpost processinput(string input)
        VAR Mytargetpost result;
        VAR num x;
        VAR num y;
        VAR string x_str;
        VAR string y_str;
        VAR num comma_pos;
        VAR bool ok;
        
        comma_pos := StrFind(input, 1, ",");
        
        IF comma_pos <= StrLen(input) THEN
        
            ! 2. Extract the X string (from index 1 up to the comma)
            x_str := StrPart(input, 1, comma_pos - 1);
            
            ! 3. Extract the Y string (from right after the comma to the end)
            y_str := StrPart(input, comma_pos + 1, StrLen(input) - comma_pos);
    
            ! 4. Convert both text strings into real numbers
            ok := StrToVal(x_str, x);
            ok := StrToVal(y_str, y);
            
            result.x := x;
            result.y := y;
        ELSE
            TPWrite "Error: No comma found in the string.";
        ENDIF
        
        RETURN result;
    ENDFUNC
    
	PROC pickandplace(num shape)
        VAR robtarget target := [[0,0,0],[1,0,0,0],[0,0,0,0],[9E9,9E9,9E9,9E9,9E9,9E9]];
		IF shape=1 THEN
			target := PHEXAGON;
        ELSEIF shape=2 THEN
            target := PSTAR;
        ELSEIF shape=3 THEN
            target := PSQUARE;
        ELSEIF shape=4 THEN
            target := PTRIANGLE;
		ENDIF
		MoveL PHOME, v1000, fine, sucker\WObj:=wobj0;
		MoveL PAPPROAC, v1000, fine, sucker\WObj:=wobj1;
		MoveL target, v1000, fine, sucker\WObj:=wobj1;
        SetDO doValet2, 1;
        WaitTime 0.5;
        
		MoveL PAPPROAC, v1000, fine, sucker\WObj:=wobj1;
		MoveL PAPPROAC, v1000, fine, sucker\WObj:=wobj2;
		MoveL target, v1000, fine, sucker\WObj:=wobj2;
        SetDO doValet2, 0;
        WaitTime 0.5;
        
		MoveL PAPPROAC, v1000, fine, sucker\WObj:=wobj2;
		MoveL PHOME, v1000, fine, sucker\WObj:=wobj0;
	ENDPROC
    
    PROC pickandplace2(num x, num y)
        VAR robtarget target := [[800.03,-177.70,813.24],[0.0162618,-0.228551,-0.973383,-0.00511577],[-1,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
		target.trans.x := x;
        target.trans.y := y;
        target.trans.z := 2.84;

		MoveL PHOME, v1000, fine, sucker\WObj:=wobj0;
		MoveL PAPPROAC, v1000, fine, sucker\WObj:=wobj1;
		MoveL target, v1000, fine, sucker\WObj:=wobj1;
        SetDO doValet2, 1;
        WaitTime 0.5;
		MoveL PAPPROAC, v1000, fine, sucker\WObj:=wobj1;
        
        
		MoveL PAPPROAC, v1000, fine, sucker\WObj:=wobj2;
		MoveL target, v1000, fine, sucker\WObj:=wobj2;
        SetDO doValet2, 0;
        WaitTime 0.5;
        
		MoveL PAPPROAC, v1000, fine, sucker\WObj:=wobj2;
		MoveL PHOME, v1000, fine, sucker\WObj:=wobj0;
	ENDPROC
ENDMODULE
