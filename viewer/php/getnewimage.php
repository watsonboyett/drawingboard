<?php
    $dir = $_REQUEST['dir'];
    $files = scandir($dir,1);
    
    $time = time();
    $file_rec = $files[0];
    $file_new = '';
    if (filemtime($dir.$file_rec) < $time-5) {
        $files_len = count($files);
        while (!is_file($file_new)) {
            $file_ind = rand(0, $files_len-2);
            $file_new = $dir.$files[$file_ind];
        }
    } else {
        $file_ind = 0;
    }

    echo $files[$file_ind]; 
?>
