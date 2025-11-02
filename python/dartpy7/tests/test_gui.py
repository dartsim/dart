import unittest

import dartpy7


class GuiBindingsTest(unittest.TestCase):

    def test_default_renderer_is_headless(self):
        self.assertTrue(dartpy7.gui_available)

        renderer = dartpy7.gui.Renderer()
        self.assertTrue(renderer.is_headless())

        options = renderer.options
        self.assertEqual(options.mode, dartpy7.gui.BackingMode.Headless)
        self.assertEqual(options.window_title, "dart7")
        self.assertEqual(options.width, 1280)
        self.assertEqual(options.height, 720)

    def test_renderer_with_custom_options(self):
        options = dartpy7.gui.RendererOptions()
        options.mode = dartpy7.gui.BackingMode.Window
        options.window_title = "demo"
        options.width = 800
        options.height = 600

        renderer = dartpy7.gui.Renderer(options)
        self.assertFalse(renderer.is_headless())

        returned = renderer.options
        self.assertEqual(returned.mode, dartpy7.gui.BackingMode.Window)
        self.assertEqual(returned.window_title, "demo")
        self.assertEqual(returned.width, 800)
        self.assertEqual(returned.height, 600)


if __name__ == "__main__":
    unittest.main()
