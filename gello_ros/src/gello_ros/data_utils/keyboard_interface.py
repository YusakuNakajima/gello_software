import pygame

NORMAL = (128, 128, 128)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
WHITE = (255, 255, 255)

KEY_START = pygame.K_s
KEY_CONTINUE = pygame.K_c
KEY_QUIT_RECORDING = pygame.K_q


class KBReset:
    def __init__(self):
        pygame.init()
        self._screen = pygame.display.set_mode((800, 800))
        self._set_color(NORMAL)

    def update(self) -> str:
        pressed_last = self._get_pressed()
        if KEY_QUIT_RECORDING in pressed_last:
            self._set_color(RED)
            # self._display_text("QUIT RECORDING")
            return "quit"

        if KEY_START in pressed_last:
            self._set_color(GREEN)
            # self._display_text("START")
            return "start"

        self._set_color(NORMAL)
        # self._display_text("READY")
        return "normal"

    def _get_pressed(self):
        pressed = []
        pygame.event.pump()
        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                pressed.append(event.key)
        return pressed

    def _set_color(self, color):
        self._screen.fill(color)
        pygame.display.flip()

    def _display_text(self, text):
        text_surface = self._font.render(text, True, WHITE)
        text_rect = text_surface.get_rect(center=(400, 400))
        self._screen.blit(text_surface, text_rect)
        pygame.display.flip()
        
def main():
    kb = KBReset()
    while True:
        state = kb.update()
        if state == "start":
            print("start")


if __name__ == "__main__":
    main()
