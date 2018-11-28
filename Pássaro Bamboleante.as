;Passaro Bamboleante
;83449 Duarte Galvao
;83539 Pedro Caldeira
;Turno segunda feira, 11:00-12:30, grupo 13
;85080 Joao Pina

; ------------------------------------------------------------------------------
; --------------------------------- CONSTANTES DO JOGO -------------------------
; ------------------------------------------------------------------------------
SP_INICIAL      EQU     FDFFh
INT_MASK_ADDR   EQU     FFFAh
INT_MASK        EQU     0000000000000111b
RANDOM_MASK     EQU     1000000000010110b
BIT_MASK        EQU     1000000000000000b
IO_DISPLAY      EQU     FFF0h
LCD_DISP_C      EQU     FFF4h
LCD_DISP_W      EQU     FFF5h
TEMP_UC         EQU     FFF6h
TEMP_PC         EQU     FFF7h
ATIVAR_TEMP     EQU     0001h
LED_DISPLAY     EQU     FFF8h
TXT_DISPLAY_C   EQU     FFFCh
TXT_DISPLAY_W   EQU     FFFEh
LIMPAR_JANELA   EQU     FFFFh
DELAY_COUNT     EQU     0001h
NIBBLE_MASK     EQU     000fh
NUM_NIBBLES     EQU     4
NIBBLE          EQU     4
BYTE            EQU     8
COL_PASSARO     EQU     20
POSI_PASSARO    EQU     0C00h
CH_ESPACO       EQU     ' '
CH_LIMITE       EQU     '-'
CH_OBSTACULO    EQU     'X'
CH_ZERO         EQU     '0'
TAM_PASSARO     EQU     2
NUM_L_SUBIR     EQU     2
DISPLAY_W       EQU     79
LIM_TOPO        EQU     0100h
ULTIMA_LINHA    EQU     1700h
NUM_BOTOES      EQU     14
LINHA           EQU     0100h
GRAVIDADE       EQU     20
INTERVALO_OBS   EQU     5 ; Espaco entre cada coluna de obstaculos
NUM_MAX_OBS     EQU     14
ESPACO_OBS      EQU     0600h ; Espaco por onde o passaro passa 
BASE_DEC        EQU     10
MAX_NIVEL       EQU     16

; Palavra de memoria que contem a variavel de contagem
                ORIG    8000h
Passaro         STR     'O>'
MsgInicL1       STR     10, 'Prepare-se'                ; Comprimento, Mensagem
MsgInicL2       STR     22, 'Prima o interruptor I1'    ; Comprimento, Mensagem
MsgFimL1        STR     11, 'Fim do Jogo'               ; Comprimento, Mensagem
MsgFimL2        STR     4,  '0000'                      ; Comprimento, Mensagem
MsgLCD          STR     10, 'Distancia:'            	; Comprimento, Mensagem

; ------------------------------------------------------------------------------
; --------------------------------- VARIAVEIS DO JOGO --------------------------
; ------------------------------------------------------------------------------
;R1=Contador da dificuldade
;R3=Ni(Random)
;R4=Posicao do cursor
;R5=(y0)Posicao da ultima vez que o passaro subiu
;R6=(t)Tempo desde a ultima vez que o passaro subiu (em ciclos do temporizador)
;R7=Interrupcoes ativadas 
;(T, I0, I1, I2, ...) (primeiro bit para T e por ai em diante)

Distancia       TAB     1			; Distancia percorrida
Nivel           TAB     1			; Nivel invertido (1-16) (16-N)
PosPassaro      TAB     1			; Posicao do passaro em virgula fixa (0C,00)
PosAPassaro     TAB     1			; Posicao anterior do passaro (em v.f.)
LinhasASubir    TAB     1			; Linhas que faltam subir
EspObstaculos   TAB     NUM_MAX_OBS	; (8043) No. maximo de obstaculos
FaseObstaculos  TAB     1		;Fase,de 0 a 5,da movimentacao dos obstaculos
NumObstaculos   TAB     1			; Numero de obstaculos existentes na janela
EstadoLEDs      TAB     1			; Conteudo dos LEDs

; ------------------------------------------------------------------------------
; ------------------------------- TABELA DE INTERRUPCOES -----------------------
; ------------------------------------------------------------------------------
                ORIG    FE00h
INT0            WORD    RInt0
INT1            WORD    RInt1			; Diminui dificuldade
INT2            WORD    RInt2			; Aumenta dificuldade
                ORIG    FE0Fh
INT15           WORD    RIntTemp

; INICIO DO CODIGO
                ORIG    0000h
                JMP     Inicio

; ------------------------------------------------------------------------------
; ------------------------------------ INTERRUPCOES ----------------------------
; ------------------------------------------------------------------------------
; RInt0: Rotina de servico a interrupcao do botao I0
;       Entradas: R7
;       Saidas: R7
;       Efeitos: ---
RInt0:          PUSH    R1
                MOV     R1, BIT_MASK
                ROR     R1, 1
                OR      R7, R1
                POP     R1
                RTI

; RInt1: Rotina de servico a interrupcao do botao I1
;       Entradas: R7
;       Saidas: R7
;       Efeitos: ---
RInt1:          PUSH    R1
                MOV     R1, BIT_MASK
                ROR     R1, 2
                OR      R7, R1
                POP     R1
                RTI

; RInt2: Rotina de servico a interrupcao do botao I2
;       Entradas: R7
;       Saidas: R7
;       Efeitos: ---
RInt2:          PUSH    R1
                MOV     R1, BIT_MASK
                ROR     R1, 3
                OR      R7, R1
                POP     R1
                RTI

; RIntTemp: Rotina de servico a interrupcao do temporizador
;       Entradas: R7
;       Saidas: R7
;       Efeitos: ---
RIntTemp:       PUSH    R1
                OR      R7, BIT_MASK
                POP     R1
                RTI
				
; ------------------------------------------------------------------------------
; --------------------------------- ROTINAS AUXILIARES -------------------------
; ------------------------------------------------------------------------------

; ApagaEcra: Rotina que apaga o ecra na sua totalidade
;       Entradas: ---
;       Saidas: ----
;       Efeitos: Altera R4, Altera M[TXT_DISPLAY_C]
ApagaEcra:      PUSH    R1
				PUSH    R2
				
                MOV     R1, CH_ESPACO
                MOV     R4, R0
				
				;Enquanto o cursor nao chegar a ultima poiscao, o ciclo continua
AE_C1:          MOV     M[TXT_DISPLAY_C], R4   
                MOV     M[TXT_DISPLAY_W], R1
                INC     R4
                MOV		R2, ULTIMA_LINHA
                ADD		R2, DISPLAY_W
                CMP		R4, R2
                BR.NZ	AE_C1
				
                POP 	R2     
                POP     R1
                RET

; DesenharLim: Rotina que desenha uma linha de um carater a escolha
;       Entradas: R4
;       Saidas: ----
;       Efeitos: Altera o R4
DesenharLim:    PUSH    R1
                PUSH    R2 ; Carater a escrever
				
                MOV     R1, DISPLAY_W
                MOV     R2, CH_LIMITE
			
				; Enquanto o cursor nao chegar a ultima coluna, o ciclo continua
DL_L1:          MOV     M[TXT_DISPLAY_C], R4  
                MOV     M[TXT_DISPLAY_W], R2
                INC     R4
                DEC     R1
                BR.NZ   DL_L1
				
                POP     R2
                POP     R1
                RET

; EscLinhaC: Rotina que escreve uma linha no meio do ecra
;       Entradas: (pilha) Linha, Mensagem
;       Saidas: ----
;       Efeitos: ---
EscLinhaC:      PUSH    R1 ; Linha
                PUSH    R2 ; Endereco da Mensagem
                PUSH    R3 ; Comprimento da mensagem
				
                MOV     R1, M[SP+6]
                MOV     R2, M[SP+5]
                MOV     R3, M[R2]
                INC     R2;Passar um a frente pois o primeiro el e o comprimento
                MOV     R4, DISPLAY_W
                SUB     R4, R3
                SHR     R4, 1
                SHL     R1, 8
                ADD     R4, R1  ; Endereco onde comeca a mensagem

ELC_C1:         MOV     M[TXT_DISPLAY_C], R4
                MOV     R1, M[R2]  ; R1 = Letra atual
                MOV     M[TXT_DISPLAY_W], R1
                INC     R2
                INC     R4
                DEC     R3
                BR.NZ   ELC_C1
				
                POP     R3
                POP     R2
                POP     R1
                RETN    2

; MataPassaro: Rotina que acaba o jogo
;       Entradas: ---
;       Saidas: ----
;       Efeitos: ---
MataPassaro:    CALL    ApagaEcra
                PUSH    12
                PUSH    MsgFimL1
				
                CALL    EscLinhaC               
                PUSH    M[Distancia]
                CALL    CalcPontuacao
                CALL    AtMsgFimL2                
                PUSH    14
                PUSH    MsgFimL2
                CALL    EscLinhaC

MP_C1:          BR      MP_C1
                RET

; ResetTemp: Rotina que ativa o temporizador
;       Entradas: ---
;       Saidas: ----
;       Efeitos: ---
ResetTemp:      PUSH    R1

                MOV     R1, DELAY_COUNT
                MOV     M[TEMP_UC], R1
                MOV     R1, ATIVAR_TEMP
                MOV     M[TEMP_PC], R1

                POP     R1
                RET

; TelaDeJogo: Rotina que desenha os limites
;       Entradas: ---
;       Saidas: ----
;       Efeitos: ---
TelaDeJogo:     MOV     R4, R0
                CALL    DesenharLim
                MOV     R4, ULTIMA_LINHA
                CALL    DesenharLim
                RET

; ------------------------------------------------------------------------------
; --------------------- CALCULO E ESCRITA DA DISTANCIA E PONTUACAO -------------
; ------------------------------------------------------------------------------
; AtMsgFimL2: Atualizar a MsgFimL2 conforme a pontuacao
;       Entradas: Pilha - pontuacao
;       Saidas: M[MsgFimL2]
;       Efeitos: ---
AtMsgFimL2:     PUSH    R1 ; Pontuacao
                PUSH    R2 ; Endereco a escrever
                PUSH    R3 ; Algarismo a escrever

                MOV     R1, M[SP+5]
                MOV     R2, MsgFimL2
                ADD     R2, M[R2]
	;Enquanto nao forem escritos na memoria todos os elementos,o ciclo continua
AMFL2_C1:       MOV     R3, BASE_DEC
                DIV     R1, R3
				ADD     R3, CH_ZERO
                MOV     M[R2], R3
                DEC     R2
                CMP     R1, R0
                BR.NZ   AMFL2_C1
                
                POP     R3
                POP     R2
                POP     R1
                RETN    1

; CalcPontuacao: Rotina que efectua o calculo da pontuacao
;       Entradas: Pilha - distancia
;       Saidas: Pilha - pontuacao
;       Efeitos: ---
CalcPontuacao:  PUSH    R1 ; Distancia
                PUSH    R2 ; DISPLAY_W - COL_PASSARO + 1
                
                ; Formula:
                ; (1) Pontuacao = 0, se R1 < R2
                ; (2) Pontuacao = (R1 - R2)/(INTERVALO_OBS + 1) + 1, se R1 >= R2
                
                ; Inicializar R1
                MOV     R1, M[SP+4]
				
                ; Inicializar R2
                MOV     R2, DISPLAY_W
                SUB     R2, COL_PASSARO
                INC     R2
				
                ; Verificar se R1 < R2
                SUB     R1, R2
                BR.NN   CP_1
				
                ; Se R1 < R2, executar (1)
                MOV     R1, R0
                BR      CP_2
                
                ; Se R1 >= R2, executar (2)
CP_1:           MOV     R2, INTERVALO_OBS
                INC     R2
                DIV     R1, R2
                INC     R1
                
                ; Devolver pontuacao
CP_2:           MOV     M[SP+4], R1

                POP     R2
                POP     R1
                RET

; EscDistancia: Rotina que escreve a distancia percorrida no display LCD.
;       Entradas: M[Distancia]
;       Saidas: ----
;       Efeitos: Altera M[LCD_DISP_C]
EscDistancia:   PUSH    R1 ; Coordenada do LCD
                PUSH    R2 ; Distancia
                PUSH    R3 ; Algarismo a ser escrito
                PUSH    R4 ; Numero de algarismos da distancia
                
                CALL    EscLCD
                
                ; Descobrir o numero de algarismos da distancia
                MOV     R2, M[Distancia]
                MOV     R4, R0
ED_C1:          INC     R4
                MOV     R3, BASE_DEC
                DIV     R2, R3
                BR.NZ   ED_C1
                
                ; Escrever a distancia no LCD
                MOV     R2, M[Distancia]
                ADD     R1, R4
ED_C2:          MOV     R3, BASE_DEC
                DIV     R2, R3
                ADD     R3, CH_ZERO
                MOV     M[LCD_DISP_C], R1
                MOV     M[LCD_DISP_W], R3
                DEC     R1
                CMP     R2, R0
                BR.NZ   ED_C2
                
                POP     R4
                POP     R3
                POP     R2
                POP     R1
                RET

; EscIO: Rotina que efectua a escrita da pontuacao
;       Entradas: ---
;       Saidas: ---
;       Efeitos: ---
EscIO:          PUSH    R1 ; Distancia
                PUSH    R2
                PUSH    R3
                PUSH    R4
				
                MOV     R2, NUM_NIBBLES
                MOV     R3, IO_DISPLAY               
                PUSH    M[Distancia]
                CALL    CalcPontuacao
                POP     R1
                
EIO_1:          MOV     R4, BASE_DEC
                DIV     R1, R4
                MOV     M[R3], R4
                INC     R3
                DEC     R2
                BR.NZ   EIO_1
				
                POP     R4
                POP     R3
                POP     R2
                POP     R1
                RET
				
; EscLCD: Rotina que escreve 'Distancia:' no display LCD:
;       Entradas: ---
;       Saidas: R1: coordenada atual do LCD
;       Efeitos: ---
;A rotina percorre cada letra da mensagem definida e vai inseri-la no lcd
EscLCD:         PUSH    R2 ; Endereco da mensagem
                PUSH    R3 ; Comprimento da mensagem
                PUSH    R4 ; Letra a ser escrita
                
                ; Limpar LCD (Escrever 1 nos bits 15 e 5)
                MOV     R1, BIT_MASK
                ROR     R1, 10
                ADD     R1, BIT_MASK
                MOV     M[LCD_DISP_C], R1
                
                MOV     R1, BIT_MASK ; Primeiro bit a 1, para ligar o display
                
                MOV     R2, MsgLCD
                MOV     R3, M[R2]
ELCD_C1:        MOV     M[LCD_DISP_C], R1   
                INC     R2
                MOV     R4, M[R2]
                MOV     M[LCD_DISP_W], R4
                INC     R1
                DEC     R3
                BR.NZ   ELCD_C1
                
                POP     R4
                POP     R3
                POP     R2
                RET

; ------------------------------------------------------------------------------
; -------------------------- MOVIMENTACAO E DESENHO DO PASSARO -----------------
; ------------------------------------------------------------------------------
; ApagaPassaro: Rotina que apaga a posicao anterior do passaro
;       Entradas: M[PosAPassaro]
;       Saidas: ----
;       Efeitos: Altera R4, M[TXT_DISPLAY_C], M[TXT_DISPLAY_W] 
ApagaPassaro:   PUSH    R1
                PUSH    R2

                MOV     R1, TAM_PASSARO
                MOV     R2, CH_ESPACO               
                ;Calcular a posicao anterior do passaro
                MOV     R4, M[PosAPassaro] 
				;Colocamos no R4 a posicao anterior do passaro
                MVBL    R4, R0             
				;Pomos a coluna a 0 para depois adiciona-la
                ADD     R4, COL_PASSARO
                
AP_C1:          MOV     M[TXT_DISPLAY_C], R4
				; Apontar o cursor para a posicao do passaro e escrever
                MOV     M[TXT_DISPLAY_W], R2
                INC     R4
                DEC     R1
                BR.NZ   AP_C1
                ;Enquanto nao eliminar o passaro repete o ciclo
                
				POP     R2
                POP     R1
                RET

; DesPassaro: Rotina que desenha o passaro
;       Entradas: ---
;       Saidas: ----
;       Efeitos: Altera R4
DesPassaro:     PUSH    R1 ; Endereco do passaro				
                PUSH    R2 ; Carater a ser escrito				
                PUSH    R3 ; Tamanho do passaro				
                ; ------R4 ; Coordenada do passaro
                PUSH    R5                
                ; Calcular posicao a desenhar o passaro
                MOV     R4, M[PosPassaro]
                MVBL    R4, R0					; Isolar a linha
                ADD     R4, COL_PASSARO                
                MOV     R1, Passaro
                MOV     R3, TAM_PASSARO

DP_C1:          MOV     R2, M[R1]               
                ; Verificar se estamos a desenhar sobre um obstaculo
                PUSH    R4
                CALL    CoordTemObs
                POP     R5
                CMP     R5, R0
                CALL.NZ MataPassaro                
                MOV     M[TXT_DISPLAY_C], R4
                MOV     M[TXT_DISPLAY_W], R2
                INC     R4
                INC     R1
                DEC     R3
                BR.NZ   DP_C1
                
                POP     R5
                POP     R3
                POP     R2
                POP     R1
                RET
                
; DescerPassaro: Rotina que desce o passaro conforme a acao da gravidade
;       Entradas: M[PosPassaro], M[PosAPassaro], R5, R6
;       Saidas: M[PosPassaro], M[PosAPassaro]
;       Efeitos: ---
DescerPassaro:  PUSH    R1
                PUSH    R2
				
                MOV     R1, M[PosPassaro]     
                MOV     R2, ULTIMA_LINHA
                SUB     R2, LINHA
                MOV     M[PosAPassaro], R1
                CMP     R1, R2
				; R1 > R2 (ultima linha)
                CALL.P  MataPassaro
                ; y = y0 + v0t + (1/2)*g*t^2
                ; v0 = 0, y0 = R5, t = R6, g = GRAVIDADE
                ; R1 = GRAVIDADE * R6 * R6 / 2 + R5
                MOV     R1, GRAVIDADE
                MOV     R2, R6
                MUL     R2, R1
                MOV     R2, R6
                MUL     R2, R1
                SHR     R1, 1
                ADD     R1, R5                                
                MOV     M[PosPassaro], R1
				
                POP     R2
                POP     R1
                RET
               
; SubirPassaro: Rotina que sobe o passaro
;       Entradas: ---
;       Saidas: ----
;       Efeitos: ---
SubirPassaro:   PUSH    R1 					;Linha em que esta o passaro
                PUSH    R2 					;Num linhas a subir

                MOV     R6, R0              ;tempo inicial e zero
                MOV     R1, M[PosPassaro]
                MOV     M[PosAPassaro], R1;Guardar a posicao anterior do passaro
                MOV     R2, NUM_L_SUBIR
                SHL     R2, BYTE;Converter 000Xh em 0X00h estava na parte das 
			   					;colunas e passa a estar na parte das linhas
                SUB     R1, R2
				; Subir o passaro NUM_L_SUBIR linhas
                SUB     R1, M[LinhasASubir]
                CMP     R1, LIM_TOPO ;Se Pos-NUM_L_SUBIR < 0100h: Pos = 0100h
                BR.N    SP_1
                MOV     R5, R1
				;guardar a posicao da ultima vez que o passaro subiu
                SUB     R2, LINHA
                ADD     M[LinhasASubir], R2
                MOV     R1, LINHA
                SUB     M[PosPassaro], R1
                BR      SP_Fim

SP_1:           MOV     R1, LIM_TOPO
                MOV     R5, R1
				; guardar a posicao da ultima vez que o passaro subiu
                MOV     M[PosPassaro], R1

SP_Fim:         POP     R2
                POP     R1
                RET

; VPosPassaro: Rotina que verifica se o passaro tem linhas a subir ou nao.
;       Entradas: ---
;       Saidas: ----
;       Efeitos: ---
VPosPassaro:    PUSH    R1

                CMP     M[LinhasASubir], R0
                BR.Z    VPP_DescPass
                MOV     R1, M[PosPassaro]
                MOV     M[PosAPassaro], R1
                MOV     R1, LINHA
                SUB     M[PosPassaro], R1
                SUB     M[LinhasASubir], R1
                BR      VPP_Fim
				
VPP_DescPass:   INC     R6
                CALL    DescerPassaro

                
VPP_Fim:        POP     R1
                RET

; ------------------------------------------------------------------------------
; ------------------------------------- OBSTACULOS -----------------------------
; ------------------------------------------------------------------------------
; DesX: Rotina que desenha um X e apaga o carater a direita.
;       Entradas: R4 - Coordenada do ecra
;       Saidas: ----
;       Efeitos: ---
DesX:           PUSH    R1

                INC     R4
                MOV     M[TXT_DISPLAY_C], R4
                MOV     R1, CH_ESPACO
                MOV     M[TXT_DISPLAY_W], R1
                DEC     R4
                ; Desenha um X
                MOV     M[TXT_DISPLAY_C], R4
                MOV     R1, CH_OBSTACULO
                MOV     M[TXT_DISPLAY_W], R1
				
                POP     R1
                RET

; ApagaUltCol: Rotina que apaga a ultima coluna da esquerda da janela de texto
;       Entradas: ---
;       Saidas: ----
;       Efeitos: Altera R4, M[TXT_DISPLAY_C], M[TXT_DISPLAY_W] 
ApagaUltCol:    PUSH    R1 ; Coordenada atual
                PUSH    R2 ; Ultima linha
                PUSH    R3 ; Espaco
                
                MOV     R1, LINHA
                MOV     R2, ULTIMA_LINHA
                MOV     R3, CH_ESPACO
AUC_C1:         MOV     M[TXT_DISPLAY_C], R1
                MOV     M[TXT_DISPLAY_W], R3
                ADD     R1, LINHA
                CMP     R1, R2
                BR.NZ   AUC_C1 ; Enquanto a linha nao for a ultima linha
                
                POP     R3
                POP     R2
                POP     R1
                RET

; DesObs: Rotina que desenha os obstaculos
;       Entradas: ---
;       Saidas: ----
;       Efeitos: ---
DesObs:         PUSH    R1 ; Numero de obstaculos
                PUSH    R2 ; Letra a desenhar
                PUSH    R3 ; Ultima linha para desenhar obs
                ; ----- R4 : Posicao do cursor              
				PUSH    R5 ; (registo auxiliar)
                PUSH    R6 ; Endereco do vetor aleatorio
                PUSH    R7 ; Numero aletorio
                
				MOV     R1, M[NumObstaculos]
                CMP     R1, R0    ;se o numero de obstaculos for 0,a rotina que
                JMP.Z   DO_Fim    ; desenha os obstaculos e passada a frente
                CALL    ApagaUltCol
                MOV     R3, ULTIMA_LINHA
                SUB     R3, LINHA
                MOV     R6, EspObstaculos
                MOV     R4, DISPLAY_W
                DEC     R4
                SUB     R4, M[FaseObstaculos]
                MOV     R7, M[R6]
				
DO_C1:          ADD     R4, LINHA
                MOV     R5, R4
                SHR     R5, BYTE
                CMP     R7, R5
                BR.NZ   DO_NaoFazerEsp
				;Ate chegar ao numero aleatorio,escreve X.
				;quando chegar, salta o n de posicoes definidas
                ADD     R4, ESPACO_OBS	 ;e continua a escrever X.
				
DO_NaoFazerEsp: CALL    DesX
                CMP     R4, R3		; Cursor<Ultima linha
                BR.N    DO_C1
                INC     R6
                MOV     R7, M[R6]
                MVBH    R4, R0		; Colocar cursor na linha zero
                SUB     R4, INTERVALO_OBS
                DEC     R4
                BR.N    DO_Fim		; Se a coluna for negativa (fora do ecra)
                DEC     R1
                BR.NZ   DO_C1
				
DO_Fim:         POP     R7
                POP     R6
                POP     R5
                POP     R3
                POP     R2
                POP     R1
                RET
				
; DeslocaObs: Rotina que desloca todos os obstaculos para a esquerda
;		Entradas: ---
;		Saidas: ---
;		Efeitos: ---
DeslocaObs:     PUSH    R1 ; Fase dos obstaculos
                PUSH    R2
                
                MOV     R1, M[FaseObstaculos]
                CMP     R1, INTERVALO_OBS
                BR.NZ   DO_1					; (Se a fase nao for a ultima)
                MOV     M[FaseObstaculos], R0
                CALL    InsereVetObs
                CALL    IncNumObs
                BR      DO_2
				
DO_1:           INC     M[FaseObstaculos]
DO_2:           INC     M[Distancia]
                CALL    EscDistancia
                CALL    EscIO
                
                POP     R2
                POP     R1
                RET
				
; IncNumObs:Rotina que incrementa o numero de obstaculos se nao atingiu o maximo
;		Entradas: M[NumObstaculos]
;		Saidas: M[NumObstaculos]
;		Efeitos: ---
IncNumObs:      PUSH    R1

                MOV     R1, NUM_MAX_OBS
                CMP     M[NumObstaculos], R1
                BR.Z    INO_Skip
                INC     M[NumObstaculos]
				
INO_Skip:       POP     R1
                RET
; InsereVetObs: Rotina que insere um novo elemento no vetor dos obstaculos
;		Entradas: ---
;		Saidas: ---
;		Efeitos: ---
InsereVetObs:   PUSH    R1
                PUSH    R2

                CALL    MemObsFrente
                CALL    RandomN               
                MOV     R1, R3					; R1 toma o valor de Ni
                ;   2 <= N_ALEATORIO <= ULTIMA_LINHA - ESPACO_OBS - 2
                ; 0 <= N_ALEATORIO - 2 <= ULTIMA_LINHA - ESPACO_OBS - 4
                MOV     R2, ULTIMA_LINHA
                SUB     R2, ESPACO_OBS
                SHR     R2, BYTE
                SUB     R2, 4
                DIV     R1, R2
                ; N_ALEATORIO = R1 + 2*LINHAS
                ADD     R2, 2
                MOV     M[EspObstaculos], R2
                
                POP     R2
                POP     R1
                RET

; MemObsFrente:Rotina que desloca os enderecos de memoria dos obs para a direita
;		Entradas: ---
;		Saidas: ---
;		Efeitos: ---
MemObsFrente:   PUSH    R1 ; Indice
                PUSH    R2 ; Endereco do objeto
                PUSH    R3
				
                MOV     R1, NUM_MAX_OBS
                SUB     R1, 2
                MOV     R2, EspObstaculos
                ADD     R2, R1
				
				; Equivale A "MOV   M[R2+1], M[R2]" 
				; passamos o R2 que esta a frente para tras				
MOF_C1:         MOV     R3, M[R2]
                INC     R2
                MOV     M[R2], R3                
                SUB     R2, 2
                DEC     R1
                BR.NN   MOF_C1
				
                POP     R3
                POP     R2
                POP     R1
                RET

; RandomN: Rotina que gera um numero aparentemente aleatorio
;       Entradas: ---
;       Saidas: ----
;       Efeitos: ---
RandomN:        TEST    R3, 0001h
                BR.NZ   RN_2
                ROR     R3, 1
                RET
				
RN_2:           XOR     R3, RANDOM_MASK
                ROR     R3, 1
                RET

; ------------------------------------------------------------------------------
; -------------------------------------- COLISOES ------------------------------
; ------------------------------------------------------------------------------

; CooordTemObs: Rotina que verifica se a coordenada dada e um obstaculo
;       Entradas: pilha - coordenada
;       Saidas: pilha - boolean: e obstaculo
;       Efeitos: ---
CoordTemObs:    PUSH    R1 ; Linha
                PUSH    R2 ; Coluna
                PUSH    R3 ; Nr de obstaculos
                PUSH    R4
                PUSH    R5 ; Endereco do vetor dos obstaculos
				
; Atraves de calculos auxiliares, conclui-se que, devido a posicao do
; passaro, ele apenas pode colidir com a decima coluna do ecra (Col 9).                
; Se houver menos de 10 obstaculos no ecra, nao pode ser obstaculo.
                MOV     R3, M[NumObstaculos]
                CMP     R3, 10
                JMP.N   CTO_False                
                MOV     R1, M[SP+7] ; coordenada a ser verificada 
                MOV     R2, R1
                SHR     R1, BYTE ;isolar a linha 
                MVBH    R2, R0 ;isolar a coluna
                MOV     R5, EspObstaculos                
; Coordenada da Col n: DISPLAY_W - 1 - Fase - n * (INTERVALO_OBS + 1)
; Coordenada da Col 9: DISPLAY_W - 10 - Fase - 9 * INTERVALO_OBS
; ------ = - 2^3 * INTERVALO_OBS - INTERVALO_OBS + DISPLAY_W - 10 - Fase
                MOV     R4, INTERVALO_OBS
                SHL     R4, 3
                ADD     R4, INTERVALO_OBS
                NEG     R4
                ADD     R4, DISPLAY_W
                SUB     R4, 10
                SUB     R4, M[FaseObstaculos]
                               
CTO_C1:         CMP     R2, R4 ; coordenada esta numa coluna com obstaculos?
                BR.Z    CTO_1               
              ;Nao esta numa coluna com obstaculos,logo nao pode ser obstaculo.
                BR      CTO_False                
                ; Coordenada esta numa coluna com obstaculos.
CTO_1:          ADD     R5, 9
                MOV     R5, M[R5] ; R5 passa a tomar o valor da sua memoria
                ; Para ser espaco: R5 - R1 <= 0 <= R5 - R1 + ESPACO_OBS
                SUB     R5, R1
                BR.P    CTO_True
                MOV     R4, ESPACO_OBS
                SHR     R4, BYTE
                ADD     R5, R4
                BR.NP   CTO_True 
                
CTO_False:      MOV     R1, R0
                BR      CTO_Fim
				
CTO_True:       MOV     R1, 1

CTO_Fim:        MOV     M[SP+7], R1
                POP     R5
				POP     R4
				POP     R3
				POP     R2
                POP     R1
                RET
; ------------------------------------------------------------------------------
; --------------------------------- DIFICULDADE DO JOGO ------------------------
; ------------------------------------------------------------------------------
; DecNivel: Rotina que diminui a dificuldade do jogo
;       Entradas: ---
;       Saidas: ----
;       Efeitos: ---
DecNivel:       PUSH    R2

                MOV     R2, MAX_NIVEL               
                CMP     M[Nivel], R2
                BR.Z    DecNivel_Skip
                INC     M[Nivel]
                SHL     M[EstadoLEDs], 1
                MOV     R2, M[EstadoLEDs]
                MOV     M[LED_DISPLAY], R2
                
DecNivel_Skip:  POP     R2
                RET
                
; IncNivel: Rotina que aumenta a dificuldade do jogo
;       Entradas: ---
;       Saidas: ----
;       Efeitos: ---
IncNivel:       PUSH    R2

                MOV     R2, 1                
                CMP     M[Nivel], R2
                BR.Z    IncNivel_Skip
                DEC     M[Nivel]
                SHRA    M[EstadoLEDs], 1
                MOV     R2, M[EstadoLEDs]
                MOV     M[LED_DISPLAY], R2               
                ; Ao diminuir o N, verificar que o contador nao ultrapassa N-1
                MOV     R2, M[Nivel]
                DEC     R2 ; R2 = N - 1
                CMP     R1, R2 ; Contador <= N - 1
                BR.NP   DecNivel_Skip
                MOV     R1, R2 ; Se ultrapassar, Contador = N - 1
                
IncNivel_Skip:  POP     R2
                RET

; TempObstaculos:Rotina que trata da temporizacao dos obs conforme o nivel
;       Entradas: ---
;       Saidas: ----
;       Efeitos: ---         
TempObstaculos: PUSH    R2

                MOV     R2, M[Nivel]
                DEC     R2               
                CMP     R1, R2
                BR.NZ   TO_SkipObs
                ; Se contador estiver no maximo
                MOV     R1, R0		; Reset ao contador
                CALL    DeslocaObs	; Execucao do deslocamento dos obstaculos
                CALL    DesObs
                BR      TO_Fim
                ; Caso contrario
				
TO_SkipObs:     INC     R1
TO_Fim:         POP     R2
                RET
; ------------------------------------------------------------------------------
; --------------------------------------- INICIO -------------------------------
; ------------------------------------------------------------------------------
; Inicio do programa
Inicio:         MOV     R1, SP_INICIAL
                MOV     SP, R1
                MOV     R1, INT_MASK
                MOV     M[INT_MASK_ADDR], R1 
				
                ; Inicializar variaveis do jogo
                MOV     M[Distancia], R0
                MOV     R2, MAX_NIVEL
                MOV     M[Nivel], R2
                MOV     R2, POSI_PASSARO
                MOV     M[PosPassaro], R2
                MOV     M[PosAPassaro], R0
				MOV     M[LinhasASubir], R0
                MOV     R2, INTERVALO_OBS
                MOV     M[FaseObstaculos], R2
                MOV     M[NumObstaculos], R0
				MOV     R2, BIT_MASK
				MOV     M[EstadoLEDs], R2
				
                ; Inicializar porto de controlo da janela de texto
                MOV     R4, LIMPAR_JANELA
                MOV     M[TXT_DISPLAY_C], R4
                CALL    ApagaEcra
                MOV     R7, R0
                MOV     R3, R0
                PUSH    12
                PUSH    MsgInicL1
                CALL    EscLinhaC
                PUSH    14
                PUSH    MsgInicL2
                CALL    EscLinhaC
                ; Inicializar variaveis do passaro
                MOV     R5, M[PosPassaro]
                MOV     R6, R0
                MOV     R2, BIT_MASK
                SHR     R2, 2
                ENI

Esperar_Inic:   INC     R3
                TEST    R7, R2
                BR.Z    Esperar_Inic
                SUB     R7, R2
                CALL    ApagaEcra                
                ; Ativar temporizador
                OR      R1, BIT_MASK
                MOV     M[INT_MASK_ADDR], R1				
                ; Desenhar limites do jogo
                CALL    TelaDeJogo               
                ; Inicializar Temporizador
                CALL    ResetTemp
                MOV     R1, R0
                ; Inicializar LEDs
                MOV     R2, M[EstadoLEDs]
				MOV     M[LED_DISPLAY], R2
                
Ciclo_1:        ENI
                MOV     R2, BIT_MASK
                DSI		
				; As interrupcoes sao desligadas durante todo o ciclo global
                ; para assegurar a ordem correta das operacoes.
                CMP     M[PosAPassaro], R0
				; Se a posicao anterior for diferente de zero,
                ; significa que houve uma subida do passaro,
				; logo tera que desenha-lo de novo.
				BR.Z    Ciclo_ITemp      
                CALL    ApagaPassaro       										   
                CALL    DesPassaro
                MOV     M[PosAPassaro], R0

Ciclo_ITemp:    TEST    R7, R2
                BR.Z    Ciclo_Int0
                CALL    ResetTemp
                CALL    VPosPassaro
                CALL    TempObstaculos
                SUB     R7, R2
Ciclo_Int0:     ROR     R2, 1
                TEST    R7, R2
                BR.Z    Ciclo_Int1
                CALL    SubirPassaro
                SUB     R7, R2
Ciclo_Int1:     ROR     R2, 1
                TEST    R7, R2
                BR.Z    Ciclo_Int2
                CALL    DecNivel
                SUB     R7, R2
Ciclo_Int2:     ROR     R2, 1
                TEST    R7, R2
                JMP.Z   Ciclo_1
                CALL    IncNivel
                SUB     R7, R2
                JMP     Ciclo_1