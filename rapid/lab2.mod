MODULE MainModule
    RECORD Mytargetpost
        num x;
        num y;
        string shape;
        num angle;
        bool valid;
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

                IF targetpost.valid THEN
                    TPWrite "X: " + ValToStr(targetpost.x) + " Y: " + ValToStr(targetpost.y) + " Shape: " + targetpost.shape + " Angle: " + ValToStr(targetpost.angle);
                    
                    pickandplace2 targetpost.x, targetpost.y, targetpost.shape, targetpost.angle;
                    
                    ! Tell the Raspberry the pick and place is finished
                    RobotClienSendMessage "DONE";
                ENDIF
            ENDIF
        ENDWHILE
        
        IF input = stopFlag THEN
            RobotClientCloseAndDisconnect;
        ENDIF
	ENDPROC
    
    ! Parse "x,y,shape,angle" (e.g. "120.5,80.2,star,35.0"). result.valid is FALSE on any error.
    FUNC Mytargetpost processinput(string input)
        VAR Mytargetpost result;
        VAR string fields{4};
        VAR num start := 1;
        VAR num comma_pos;
        VAR bool ok;
        
        result.valid := FALSE;
        
        ! Split the input on commas into exactly 4 fields
        FOR i FROM 1 TO 3 DO
            comma_pos := StrFind(input, start, ",");
            IF comma_pos > StrLen(input) THEN
                TPWrite "Error: expected x,y,shape,angle but got: " + input;
                RETURN result;
            ENDIF
            fields{i} := StrPart(input, start, comma_pos - start);
            start := comma_pos + 1;
        ENDFOR
        fields{4} := StrPart(input, start, StrLen(input) - start + 1);
        
        ! Convert the numeric fields; the shape stays as text
        ok := StrToVal(fields{1}, result.x);
        IF NOT ok THEN
            TPWrite "Error: invalid x: " + fields{1};
            RETURN result;
        ENDIF
        ok := StrToVal(fields{2}, result.y);
        IF NOT ok THEN
            TPWrite "Error: invalid y: " + fields{2};
            RETURN result;
        ENDIF
        result.shape := fields{3};
        ok := StrToVal(fields{4}, result.angle);
        IF NOT ok THEN
            TPWrite "Error: invalid angle: " + fields{4};
            RETURN result;
        ENDIF
        
        result.valid := TRUE;
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
    
    ! Pick the object at (x, y) in wobj1, rotated about the wobj Z axis by angle (degrees),
    ! then place it at the predefined position of its shape in wobj2.
    PROC pickandplace2(num x, num y, string shape, num angle)
        VAR robtarget target := [[800.03,-177.70,813.24],[0.0162618,-0.228551,-0.973383,-0.00511577],[-1,0,0,0],[9E+09,9E+09,9E+09,9E+09,9E+09,9E+09]];
        VAR robtarget placetarget;

        IF shape = "hexagon" THEN
            placetarget := PHEXAGON;
        ELSEIF shape = "star" THEN
            placetarget := PSTAR;
        ELSEIF shape = "square" THEN
            placetarget := PSQUARE;
        ELSEIF shape = "triangle" THEN
            placetarget := PTRIANGLE;
        ELSE
            TPWrite "Error: unknown shape: " + shape;
            RETURN;
        ENDIF

        target.trans.x := x;
        target.trans.y := y;
        target.trans.z := 2.84;
        ! Rotate the tool about the Z axis of the work object (pre-multiply)
        target.rot := OrientZYX(angle, 0, 0) * target.rot;

        MoveL PHOME, v1000, fine, sucker\WObj:=wobj0;
        MoveL PAPPROAC, v1000, fine, sucker\WObj:=wobj1;
        MoveL target, v1000, fine, sucker\WObj:=wobj1;
        SetDO doValet2, 1;
        WaitTime 0.5;
        MoveL PAPPROAC, v1000, fine, sucker\WObj:=wobj1;

        MoveL PAPPROAC, v1000, fine, sucker\WObj:=wobj2;
        MoveL placetarget, v1000, fine, sucker\WObj:=wobj2;
        SetDO doValet2, 0;
        WaitTime 0.5;

        MoveL PAPPROAC, v1000, fine, sucker\WObj:=wobj2;
        MoveL PHOME, v1000, fine, sucker\WObj:=wobj0;
    ENDPROC
ENDMODULE
