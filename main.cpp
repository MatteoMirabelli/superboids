
      // incrementa counter
      (counter == 1000) ? counter = 0 : ++counter;
    }
    return EXIT_SUCCESS;
  } catch (std::exception& e) {
    // eccezione standard (x metodi SL)
    std::cerr << e.what();
  } catch (...) {
    // eccezione generica
    std::cerr << "Unknown exception";
  }
}