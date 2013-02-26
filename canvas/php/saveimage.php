<?php
    $img = $_REQUEST['img'];
    $name = $_REQUEST['name'];
    $loc = $_REQUEST['loc'];
    
    $imge = explode('base64,', $img);
    $imgd = base64_decode($imge[1]);
    file_put_contents("$loc{$name}", $imgd);
?>
