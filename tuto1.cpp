
int main(int argc, char* argv[]){


    bool proceed = true;
    cv::Mat frame(10, 10, 0);

    while(proceed) {
        cv::imshow("tuto1", frame);

        char k = (char) cv::waitKey(10);
        switch (k) {
            case 'q':
                std::cout << "EMERGENCY BUTTON" << std::endl;
                proceed = false;
                break;
        }
    }
}